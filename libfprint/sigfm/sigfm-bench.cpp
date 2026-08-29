/*
 * SIGFM 1:N matching benchmark
 * Copyright (C) 2026 Sergey Subbotin <ssubbotin@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * Measures FAR/FRR of the shipped SIGFM implementation by calling the same
 * public entry points a driver does -- sigfm_extract() and sigfm_match_score()
 * -- rather than a reimplementation of the algorithm.
 *
 * A large-area reference print is reduced to a sequence of small windows that
 * approximate what a small-area sensor sees: a random placement (offset plus
 * rotation) of the finger over the capture window, optionally upscaled the way
 * a driver does before extraction. Each subject contributes a gallery of
 * `--stages` windows (standing in for a multi-stage enrolment) and `--probes`
 * further windows used as verification attempts. A probe is scored against the
 * whole gallery, taking the best score, which is what fpi_print_sigfm_match()
 * does.
 *
 * Every placement is drawn from its own RNG stream, seeded from the master
 * seed, the dataset label, the finger id, the role (gallery or probe) and the
 * window index. Any single window is therefore regenerable in isolation, which
 * is what lets --export-trials write a replay subset without perturbing the
 * sweep: the exported pixels are byte-identical to the ones scored, and the
 * scores are identical with and without the flag.
 *
 * Only interior placements are used: a window whose rotated footprint would
 * leave the source image is redrawn, so no synthetic edge between print and
 * fill ever reaches the detector. A window the device would have refused --
 * fewer than 25 keypoints, the gate in fp-image.c -- is redrawn as well, the
 * way the device asks for the finger again; a gallery that cannot be built
 * this way drops the subject, and a probe that cannot be captured is recorded
 * as a failed capture rather than as a score.
 *
 * Output is a CSV of `is_genuine,score` compatible with wl2776's
 * plot_sigfm_evaluations.py (https://gitlab.com/wl2776/sigfm-eval), plus the
 * optional per-trial TSV (--scores) and replay subset (--export-trials) that
 * tests/benchmark-matching.py consumes.
 *
 * Input images are binary PGM (P5, maxval 255), one subject per file, named
 * so that the subject id is the text before the first underscore -- the
 * FVC/NIST convention, e.g. `101_1.pgm`, `101_2.pgm`, `102_1.pgm`. One run
 * reads one dataset directory; a directory tree that hides further .pgm files
 * in subdirectories is rejected rather than silently pooled. Convert a
 * dataset with ImageMagick:
 *
 *     mogrify -format pgm -compress none *.tif
 */

#include "sigfm.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

/* Stamped by the build system when available; the banner says "unknown"
 * otherwise. Build with -DSIGFM_BENCH_COMMIT='"<sha>"' to pin a report to a
 * revision. */
#ifndef SIGFM_BENCH_COMMIT
#define SIGFM_BENCH_COMMIT "unknown"
#endif

namespace {

/* ------------------------------------------------------------------ *
 * Images                                                              *
 * ------------------------------------------------------------------ */

struct Image {
    int w = 0, h = 0;
    std::vector<unsigned char> px;
    bool valid() const { return w > 0 && h > 0; }
    unsigned char at(int x, int y) const
    {
        return px[static_cast<size_t>(y) * w + x];
    }
};

Image read_pgm(const std::string& path)
{
    Image im;
    std::ifstream f(path, std::ios::binary);

    if (!f)
        return im;

    std::string magic;
    f >> magic;
    if (magic != "P5")
        return {};

    auto next_int = [&f]() {
        int v = -1;
        while (f) {
            int c = f.peek();
            if (c == '#') {
                std::string line;
                std::getline(f, line);
                continue;
            }
            if (std::isspace(c)) {
                f.get();
                continue;
            }
            f >> v;
            break;
        }
        return v;
    };

    int w = next_int(), h = next_int(), maxv = next_int();
    if (w <= 0 || h <= 0 || maxv != 255)
        return {};
    f.get();

    im.w = w;
    im.h = h;
    im.px.resize(static_cast<size_t>(w) * h);
    f.read(reinterpret_cast<char*>(im.px.data()),
           static_cast<std::streamsize>(w) * h);
    if (!f)
        return {};
    return im;
}

bool write_pgm(const std::string& path, const Image& im)
{
    /* Unlink first: fopen("wb") truncates in place, which would write through
     * a hard link an earlier export left behind at this name. */
    unlink(path.c_str());

    std::FILE* f = std::fopen(path.c_str(), "wb");

    if (!f)
        return false;

    std::fprintf(f, "P5\n%d %d\n255\n", im.w, im.h);
    const bool ok = std::fwrite(im.px.data(), 1, im.px.size(), f) == im.px.size();
    std::fclose(f);
    return ok;
}

/* One placement of the finger over the capture window: rotate about the
 * requested centre and sample a w*h window, then replicate each pixel `scale`
 * times as the fpc1022 driver does before handing the image to SIGFM. Only
 * interior placements reach this function, so the initial fill is never seen
 * in the result. */
Image sample_window(const Image& src, double cx, double cy, double theta, int w,
                    int h, int scale)
{
    Image out;

    out.w = w * scale;
    out.h = h * scale;
    out.px.assign(static_cast<size_t>(out.w) * out.h, 255);

    const double ct = std::cos(theta), st = std::sin(theta);

    for (int j = 0; j < out.h; j++) {
        for (int i = 0; i < out.w; i++) {
            /* Undo the upscale, then map into the source frame. */
            const double lx = (i / scale) - w / 2.0;
            const double ly = (j / scale) - h / 2.0;
            const double sx = cx + lx * ct - ly * st;
            const double sy = cy + lx * st + ly * ct;
            const int ix = static_cast<int>(std::lround(sx));
            const int iy = static_cast<int>(std::lround(sy));

            if (ix >= 0 && ix < src.w && iy >= 0 && iy < src.h)
                out.px[static_cast<size_t>(j) * out.w + i] = src.at(ix, iy);
        }
    }
    return out;
}

/* ------------------------------------------------------------------ *
 * SHA-256 (FIPS 180-4)                                                *
 * ------------------------------------------------------------------ */

/* Vendored so the benchmark keeps its current dependency set: it is a test
 * tool, never installed, and pulling a crypto library into the build for one
 * digest of a pixel buffer is not worth it. Public domain, transcribed from
 * the published pseudocode. */

const uint32_t sha256_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

uint32_t rotr32(uint32_t x, int n)
{
    return (x >> n) | (x << (32 - n));
}

std::string sha256_hex(const unsigned char* data, size_t len)
{
    uint32_t h[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                      0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };

    /* message, a 0x80 byte, zero padding, then the length in bits (BE) */
    std::vector<unsigned char> m(data, data + len);
    m.push_back(0x80);
    while (m.size() % 64 != 56)
        m.push_back(0);
    const uint64_t bits = static_cast<uint64_t>(len) * 8;
    for (int i = 7; i >= 0; i--)
        m.push_back(static_cast<unsigned char>((bits >> (i * 8)) & 0xff));

    for (size_t off = 0; off < m.size(); off += 64) {
        uint32_t w[64];

        for (int i = 0; i < 16; i++)
            w[i] = (static_cast<uint32_t>(m[off + 4 * i]) << 24) |
                   (static_cast<uint32_t>(m[off + 4 * i + 1]) << 16) |
                   (static_cast<uint32_t>(m[off + 4 * i + 2]) << 8) |
                   static_cast<uint32_t>(m[off + 4 * i + 3]);

        for (int i = 16; i < 64; i++) {
            const uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^
                                (w[i - 15] >> 3);
            const uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^
                                (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];

        for (int i = 0; i < 64; i++) {
            const uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
            const uint32_t ch = (e & f) ^ (~e & g);
            const uint32_t t1 = hh + S1 + ch + sha256_k[i] + w[i];
            const uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t t2 = S0 + maj;

            hh = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }

        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += hh;
    }

    char out[65];
    for (int i = 0; i < 8; i++)
        std::snprintf(out + i * 8, 9, "%08x", h[i]);
    return std::string(out, 64);
}

/* ------------------------------------------------------------------ *
 * Per-trial RNG                                                       *
 * ------------------------------------------------------------------ */

/* SplitMix64 (Steele/Lea/Flood), the canonical constants. Used only as a
 * mixing function: it turns the tuple (seed, dataset, finger, role, index)
 * into one well-spread word that seeds this window's mt19937. */
uint64_t splitmix64(uint64_t x)
{
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

uint64_t fnv1a(const std::string& s)
{
    uint64_t h = 1469598103934665603ULL;

    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}

enum Role {
    ROLE_GALLERY = 0,
    ROLE_PROBE = 1,
};

/* A uniform double in [0, 1) assembled from the generator's own bits: two
 * 32-bit draws, the top 53 of them scaled by 2^-53. std::uniform_real_-
 * distribution computes the same quantity, but its mapping from generator
 * output to doubles is library-defined, so the same seed would place windows
 * differently under another standard library. The subset picker below avoids
 * the integer distributions for the same reason. */
double next_unit(std::mt19937& rng)
{
    const uint64_t hi = static_cast<uint64_t>(rng()) << 32;
    const uint64_t lo = rng();

    return static_cast<double>((hi | lo) >> 11) * 0x1.0p-53;
}

/* Uniform in [lo, hi). */
double next_uniform(std::mt19937& rng, double lo, double hi)
{
    return lo + next_unit(rng) * (hi - lo);
}

/* The generator for one window. Nothing about it depends on how many windows
 * were drawn before, so a trial can be regenerated on its own -- that is what
 * makes --export-trials a pure observer of the sweep. */
std::mt19937 trial_rng(unsigned seed, const std::string& dataset,
                       const std::string& finger, int role, int index)
{
    uint64_t h = seed;

    h = splitmix64(h ^ fnv1a(dataset));
    h = splitmix64(h ^ fnv1a(finger));
    h = splitmix64(h ^ (static_cast<uint64_t>(role) << 32 |
                        static_cast<uint32_t>(index)));
    return std::mt19937(static_cast<uint32_t>(h ^ (h >> 32)));
}

/* ------------------------------------------------------------------ *
 * Options                                                             *
 * ------------------------------------------------------------------ */

struct Opts {
    std::string dir;
    std::string out = "sigfm-bench.csv";
    std::string dataset_id;
    std::string scores;
    std::string export_dir;
    bool dataset_id_given = false;
    int width = 112, height = 88, scale = 2;
    int stages = 5, probes = 5;
    int max_subjects = 0;    /* 0 = all */
    double max_shift = 0.25; /* fraction of the window */
    double max_rot_deg = 25.0;
    unsigned seed = 12345;
    int threshold = 10;
};

/* ------------------------------------------------------------------ *
 * Placement                                                           *
 * ------------------------------------------------------------------ */

struct Placement {
    double cx = 0, cy = 0, theta = 0;
    bool valid = false;
};

/* The four corners of the un-upscaled window, in the coordinates
 * sample_window() actually samples. */
bool window_inside(const Image& src, const Placement& p, int w, int h)
{
    const double ct = std::cos(p.theta), st = std::sin(p.theta);
    const double lx[2] = { -w / 2.0, (w - 1) - w / 2.0 };
    const double ly[2] = { -h / 2.0, (h - 1) - h / 2.0 };

    for (int a = 0; a < 2; a++) {
        for (int b = 0; b < 2; b++) {
            const double sx = p.cx + lx[a] * ct - ly[b] * st;
            const double sy = p.cy + lx[a] * st + ly[b] * ct;

            /* Interior in floating point: every sampled point lies in the
             * convex hull of the corners, so rounding cannot escape the
             * image either. */
            if (sx < 0.0 || sx > src.w - 1 || sy < 0.0 || sy > src.h - 1)
                return false;
        }
    }
    return true;
}

/* One draw from the stream. A placement is usable only when the whole rotated
 * window lies inside the source: filling the outside with a constant would
 * hand SIFT a synthetic straight edge, which scores as structure it did not
 * measure. */
Placement next_placement(std::mt19937& rng, const Image& src, const Opts& o)
{
    const double sx = o.width * o.max_shift, sy = o.height * o.max_shift;
    const double rot = o.max_rot_deg * M_PI / 180.0;
    Placement p;

    p.cx = src.w / 2.0 + next_uniform(rng, -sx, sx);
    p.cy = src.h / 2.0 + next_uniform(rng, -sy, sy);
    p.theta = next_uniform(rng, -rot, rot);
    p.valid = window_inside(src, p, o.width, o.height);
    return p;
}

/* The device's capture quality gate: an extraction that returns nothing, or
 * one with fewer than 25 keypoints, never becomes an image the matcher
 * sees -- the driver is asked for the finger again. Source of truth for the
 * 25 threshold: fp_image_sigfm_extract_thread_func() in libfprint/fp-image.c
 * (cited by function name, not by line, since line numbers rot). Duplicated
 * here because the bench calls sigfm_extract() directly and would otherwise
 * score windows no device would ever hand to the matcher. */
const int MIN_KEYPOINTS = 25;

/* Draws per window, spent on both rejection reasons: a placement that falls
 * outside the source, and a capture the device would have refused. */
const int CAPTURE_DRAWS = 100;

enum CaptureStatus {
    CAPTURE_OK = 0,
    CAPTURE_NO_PLACEMENT, /* never landed wholly inside the source */
    CAPTURE_LOW_QUALITY,  /* landed inside, never enough keypoints */
};

struct Capture {
    CaptureStatus status = CAPTURE_NO_PLACEMENT;
    SigfmImgInfo* info = nullptr; /* the caller owns it when status is OK */
    int draw = -1;                /* the draw the window came from */
};

/* One window for (finger, role, index): draw, sample, extract, and if the
 * device would have rejected the capture, draw again from the same stream --
 * which is what the device does when it reports a retry. `draw` records which
 * draw won, so the export can reproduce the placement without extracting
 * again. On exhaustion `draw` still points at the first interior window, the
 * one the device would have seen first, so a failed capture is exportable. */
Capture capture_window(const Image& full, const Opts& o,
                       const std::string& finger, int role, int index)
{
    std::mt19937 rng = trial_rng(o.seed, o.dataset_id, finger, role, index);
    Capture c;

    for (int draw = 0; draw < CAPTURE_DRAWS; draw++) {
        const Placement p = next_placement(rng, full, o);

        if (!p.valid)
            continue;

        if (c.draw < 0) {
            c.draw = draw;
            c.status = CAPTURE_LOW_QUALITY;
        }

        const Image win = sample_window(full, p.cx, p.cy, p.theta, o.width,
                                        o.height, o.scale);
        SigfmImgInfo* info = sigfm_extract(win.px.data(), win.w, win.h);

        if (info && sigfm_keypoints_count(info) >= MIN_KEYPOINTS) {
            c.status = CAPTURE_OK;
            c.info = info;
            c.draw = draw;
            return c;
        }
        sigfm_free_info(info);
    }
    return c;
}

/* Replay the placement of one numbered draw. The interior test is pure
 * geometry, so the draw sequence reproduces without extracting anything. */
Image window_at(const Image& full, const Opts& o, const std::string& finger,
                int role, int index, int draw)
{
    std::mt19937 rng = trial_rng(o.seed, o.dataset_id, finger, role, index);

    for (int i = 0; i <= draw; i++) {
        const Placement p = next_placement(rng, full, o);

        if (i == draw && p.valid)
            return sample_window(full, p.cx, p.cy, p.theta, o.width, o.height,
                                 o.scale);
    }
    return Image();
}

/* ------------------------------------------------------------------ *
 * Dataset hygiene                                                     *
 * ------------------------------------------------------------------ */

bool has_pgm_suffix(const std::string& n)
{
    return n.size() >= 5 && n.compare(n.size() - 4, 4, ".pgm") == 0;
}

/* The single filename parser: the finger id is the text before the first
 * underscore (FVC `X_Y.tif` names the finger X and the impression Y). */
std::string subject_of(const std::string& fname)
{
    const auto us = fname.find('_');

    return us == std::string::npos ? fname : fname.substr(0, us);
}

bool is_dir(const std::string& path)
{
    struct stat st;

    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

/* Any .pgm below `path`, at any depth. */
bool tree_has_pgm(const std::string& path)
{
    DIR* d = opendir(path.c_str());

    if (!d)
        return false;

    bool found = false;
    while (dirent* e = readdir(d)) {
        const std::string n = e->d_name;

        if (n == "." || n == "..")
            continue;

        const std::string sub = path + "/" + n;
        if (has_pgm_suffix(n) && !is_dir(sub)) {
            found = true;
            break;
        }
        if (is_dir(sub) && tree_has_pgm(sub)) {
            found = true;
            break;
        }
    }
    closedir(d);
    return found;
}

/* The files an export owns, and the only ones it may delete. */
bool is_export_artifact(const std::string& n)
{
    if (n == "manifest.tsv")
        return true;

    return n.compare(0, 5, "trial") == 0 && has_pgm_suffix(n) &&
           n.find('_') != std::string::npos;
}

std::string basename_of(const std::string& path)
{
    std::string p = path;

    while (p.size() > 1 && p.back() == '/')
        p.pop_back();

    const auto slash = p.find_last_of('/');
    const std::string base = slash == std::string::npos ? p
                                                        : p.substr(slash + 1);

    return base.empty() ? std::string("dataset") : base;
}

/* ------------------------------------------------------------------ *
 * Statistics                                                          *
 * ------------------------------------------------------------------ */

struct Interval {
    double lo = 0, hi = 0;
};

/* Wilson score interval, 95%. Preferred over the normal approximation
 * because the counts that matter here are tiny relative to N. */
Interval wilson95(long k, long n)
{
    const double z = 1.959963984540054;

    if (n <= 0)
        return Interval();

    const double p = static_cast<double>(k) / n;
    const double z2 = z * z;
    const double denom = 1.0 + z2 / n;
    const double centre = (p + z2 / (2.0 * n)) / denom;
    const double half = (z / denom) *
                        std::sqrt(p * (1.0 - p) / n +
                                  z2 / (4.0 * static_cast<double>(n) * n));
    Interval iv;

    iv.lo = centre - half < 0.0 ? 0.0 : centre - half;
    iv.hi = centre + half > 1.0 ? 1.0 : centre + half;
    return iv;
}

/* A rate of zero is not evidence that the rate is zero: with no observed
 * event in n trials the 95% upper bound is 3/n. */
void print_rate(const char* label, long k, long n)
{
    if (n <= 0) {
        std::printf("%-20s: 0/0  (no trials)\n", label);
        return;
    }
    if (k == 0) {
        /* Rule of three, clamped: below three trials the bound says only that
         * the rate could be anything. */
        std::printf("%-20s: 0/%ld  (95%% upper bound %.3f%%)\n", label, n,
                    std::min(300.0 / n, 100.0));
        return;
    }

    const Interval iv = wilson95(k, n);

    std::printf("%-20s: %ld/%ld  (95%% CI %.3f%%..%.3f%%)\n", label, k, n,
                iv.lo * 100.0, iv.hi * 100.0);
}

/* Nearest-rank percentile over an ascending vector. */
int percentile(const std::vector<int>& sorted, double q)
{
    if (sorted.empty())
        return 0;

    size_t idx = static_cast<size_t>(std::ceil(q * sorted.size()));

    if (idx == 0)
        idx = 1;
    if (idx > sorted.size())
        idx = sorted.size();
    return sorted[idx - 1];
}

void print_percentiles(const char* label, const std::vector<int>& sorted)
{
    if (sorted.empty()) {
        std::printf("%-20s: (none)\n", label);
        return;
    }
    std::printf("%-20s: p50 %d  p90 %d  p99 %d  p99.9 %d  max %d\n", label,
                percentile(sorted, 0.50), percentile(sorted, 0.90),
                percentile(sorted, 0.99), percentile(sorted, 0.999),
                sorted.back());
}

/* ------------------------------------------------------------------ *
 * Trials                                                              *
 * ------------------------------------------------------------------ */

struct Sample {
    SigfmImgInfo* info = nullptr; /* null when the capture was refused */
    std::string subject;
    std::string path;
    int index = 0; /* stage or probe number, the RNG's `index` */
    int draw = 0;  /* the accepted draw, for regenerating the window */
};

struct Trial {
    long id = 0;
    bool genuine = false;
    bool failed = false;
    int probe = 0;   /* index into `probes` */
    int gallery = 0; /* index into `galleries` */
    int best = 0;
    std::vector<int> cmp; /* per-stage scores, gallery order */
};

void usage(const char* argv0)
{
    std::fprintf(
        stderr,
        "usage: %s -i <dir-of-pgm> [options]\n"
        "  -o <file>       output CSV (default sigfm-bench.csv)\n"
        "  -W <px> -H <px> capture window (default 112x88)\n"
        "  --scale <n>     driver upscale factor (default 2)\n"
        "  --stages <n>    gallery windows per subject (default 5)\n"
        "  --probes <n>    probe windows per subject (default 5)\n"
        "  --subjects <n>  limit number of subjects\n"
        "  --rot <deg>     max random rotation (default 25)\n"
        "  --shift <frac>  max random shift, fraction of window (default "
        "0.25)\n"
        "  --threshold <n> score threshold for the summary (default 10)\n"
        "  --seed <n>      RNG seed (default 12345)\n"
        "  --dataset-id <s> dataset label used for identity and in the\n"
        "                  per-trial RNG (default: basename of -i)\n"
        "  --scores <file> per-trial score dump, TSV\n"
        "  --export-trials <dir>\n"
        "                  write the replay subset (PGM windows plus\n"
        "                  manifest.tsv); requires --dataset-id\n",
        argv0);
}

} // namespace

int main(int argc, char** argv)
{
    Opts o;

    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto need = [&](const char* what) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s needs a value\n", what);
                std::exit(2);
            }
            return std::string(argv[++i]);
        };

        if (a == "-i")
            o.dir = need("-i");
        else if (a == "-o")
            o.out = need("-o");
        else if (a == "-W")
            o.width = std::stoi(need("-W"));
        else if (a == "-H")
            o.height = std::stoi(need("-H"));
        else if (a == "--scale")
            o.scale = std::stoi(need("--scale"));
        else if (a == "--stages")
            o.stages = std::stoi(need("--stages"));
        else if (a == "--probes")
            o.probes = std::stoi(need("--probes"));
        else if (a == "--subjects")
            o.max_subjects = std::stoi(need("--subjects"));
        else if (a == "--rot")
            o.max_rot_deg = std::stod(need("--rot"));
        else if (a == "--shift")
            o.max_shift = std::stod(need("--shift"));
        else if (a == "--threshold")
            o.threshold = std::stoi(need("--threshold"));
        else if (a == "--seed")
            o.seed = std::stoul(need("--seed"));
        else if (a == "--dataset-id") {
            o.dataset_id = need("--dataset-id");
            o.dataset_id_given = true;
        }
        else if (a == "--scores")
            o.scores = need("--scores");
        else if (a == "--export-trials")
            o.export_dir = need("--export-trials");
        else {
            usage(argv[0]);
            return 2;
        }
    }

    if (o.dir.empty()) {
        usage(argv[0]);
        return 2;
    }
    if (o.scale < 1 || o.width < 1 || o.height < 1 || o.stages < 1 ||
        o.probes < 1) {
        std::fprintf(stderr, "window, scale, stages and probes must be >= 1\n");
        return 2;
    }

    /* The manifest records the dataset half of the (dataset, finger) identity,
     * so the replay must not depend on a directory name that happened to be
     * on the command line. */
    if (!o.export_dir.empty() && !o.dataset_id_given) {
        std::fprintf(stderr, "--export-trials requires --dataset-id\n");
        return 2;
    }
    if (o.dataset_id.empty())
        o.dataset_id = basename_of(o.dir);

    if (!is_dir(o.dir)) {
        std::fprintf(stderr, "cannot open %s\n", o.dir.c_str());
        return 1;
    }

    /* Collect one representative file per subject, and refuse a tree that
     * hides a second dataset below this one. */
    std::map<std::string, std::string> by_subject;
    if (DIR* d = opendir(o.dir.c_str())) {
        std::vector<std::string> names;
        std::string nested;

        while (dirent* e = readdir(d)) {
            const std::string n = e->d_name;

            if (n == "." || n == "..")
                continue;

            const std::string full = o.dir + "/" + n;
            if (is_dir(full)) {
                if (nested.empty() && tree_has_pgm(full))
                    nested = full;
                continue;
            }
            if (has_pgm_suffix(n))
                names.push_back(n);
        }
        closedir(d);

        if (!nested.empty()) {
            std::fprintf(stderr,
                         "hard error: one run, one dataset directory "
                         "(%s also holds .pgm files)\n",
                         nested.c_str());
            return 1;
        }

        /* readdir order is whatever the filesystem feels like; sorting first
         * makes the representative impression of a finger the same on every
         * machine. */
        std::sort(names.begin(), names.end());
        for (const std::string& n : names)
            by_subject.emplace(subject_of(n), o.dir + "/" + n);
    }
    else {
        std::fprintf(stderr, "cannot open %s\n", o.dir.c_str());
        return 1;
    }

    if (by_subject.empty()) {
        std::fprintf(stderr, "no .pgm files in %s\n", o.dir.c_str());
        return 1;
    }

    std::vector<std::vector<Sample>> galleries;
    std::vector<Sample> probes;
    int subjects = 0, skipped = 0, failed_captures = 0;

    for (const auto& kv : by_subject) {
        if (o.max_subjects && subjects >= o.max_subjects)
            break;

        const Image full = read_pgm(kv.second);
        if (!full.valid()) {
            std::fprintf(stderr, "skip (unreadable) %s\n", kv.second.c_str());
            skipped++;
            continue;
        }
        if (full.w < o.width || full.h < o.height) {
            std::fprintf(stderr, "skip (too small) %s\n", kv.second.c_str());
            skipped++;
            continue;
        }

        std::vector<Sample> gal;
        const char* skip_reason = nullptr;

        for (int i = 0; i < o.stages; i++) {
            const Capture c = capture_window(full, o, kv.first, ROLE_GALLERY,
                                             i);

            /* A stage the device would keep asking to rescan is an enrolment
             * that never completes, not a low score. */
            if (c.status != CAPTURE_OK) {
                skip_reason = c.status == CAPTURE_NO_PLACEMENT
                                  ? "no interior placement"
                                  : "enrolment failed";
                break;
            }
            gal.push_back({ c.info, kv.first, kv.second, i, c.draw });
        }

        /* A gallery that could not be built is an enrolment failure, not a
         * matching result; drop the subject rather than bias the scores. */
        if (skip_reason) {
            for (auto& s : gal)
                sigfm_free_info(s.info);
            std::fprintf(stderr, "skip (%s) %s\n", skip_reason,
                         kv.second.c_str());
            skipped++;
            continue;
        }

        std::vector<Sample> mine;
        for (int i = 0; i < o.probes; i++) {
            const Capture c = capture_window(full, o, kv.first, ROLE_PROBE, i);

            if (c.status == CAPTURE_NO_PLACEMENT) {
                skip_reason = "no interior placement";
                break;
            }

            /* A probe the device would have refused still belongs in the
             * sweep, as a failed capture rather than as a score: the replay
             * has to reach the same verdict on the same pixels. */
            if (c.status != CAPTURE_OK)
                failed_captures++;
            mine.push_back({ c.info, kv.first, kv.second, i, c.draw });
        }

        if (skip_reason) {
            for (auto& s : gal)
                sigfm_free_info(s.info);
            for (auto& s : mine)
                sigfm_free_info(s.info);
            std::fprintf(stderr, "skip (no interior placement) %s\n",
                         kv.second.c_str());
            skipped++;
            continue;
        }

        probes.insert(probes.end(), mine.begin(), mine.end());
        galleries.push_back(std::move(gal));
        subjects++;
        std::fprintf(stderr, "\renrolled %d subjects", subjects);
    }
    std::fprintf(stderr, "\n");

    if (galleries.empty() || probes.empty()) {
        std::fprintf(stderr, "nothing to compare\n");
        return 1;
    }

    std::FILE* csv = std::fopen(o.out.c_str(), "w");
    if (!csv) {
        std::fprintf(stderr, "cannot write %s\n", o.out.c_str());
        return 1;
    }
    std::fprintf(csv, "is_genuine,score\n");

    const bool keep_trials = !o.scores.empty() || !o.export_dir.empty();
    std::vector<Trial> trials;
    std::vector<int> gen_scores, imp_scores;
    long genuine_n = 0, genuine_hit = 0, impostor_n = 0, impostor_hit = 0;
    long failed_trials = 0;
    long long cmp_count = 0;
    long trial_id = 0;

    /* Probes outer, galleries inner: that order defines trial_id. */
    for (size_t pi = 0; pi < probes.size(); pi++) {
        const Sample& p = probes[pi];

        for (size_t gi = 0; gi < galleries.size(); gi++) {
            const std::vector<Sample>& gal = galleries[gi];
            Trial t;

            t.id = trial_id++;
            t.probe = static_cast<int>(pi);
            t.gallery = static_cast<int>(gi);
            t.genuine = (gal.front().subject == p.subject);

            /* Best score across the gallery, as fpi_print_sigfm_match does.
             *
             * Argument order is load-bearing: sigfm_match_score() runs
             * knnMatch() with its first argument as the query set, which is
             * not symmetric. fpi_print_sigfm_match() passes the enrolled
             * gallery entry first and the fresh probe second -- the reverse
             * of what sigfm.h documents -- so the sweep calls it the same
             * way round. Measuring the other direction would report numbers
             * no device produces. */
            const bool captured = p.info != nullptr;
            int best = 0;

            for (const auto& g : gal) {
                /* A capture the device refused never reaches the matcher.
                 * Recording a negative score marks the trial as a failed
                 * capture, which is what the replay checks for. */
                const int s = captured ? sigfm_match_score(g.info, p.info) : -1;

                if (captured)
                    cmp_count++;
                if (s < 0)
                    t.failed = true;
                if (s > best)
                    best = s;
                if (keep_trials)
                    t.cmp.push_back(s);
            }
            t.best = best;

            /* A failed capture is an acquisition failure, not a decision:
             * counting it as a reject would report an FRR the device never
             * produces, since the device simply asks for the finger again. */
            if (!captured)
                failed_trials++;
            else {
                std::fprintf(csv, "%d,%d\n", t.genuine ? 1 : 0, best);

                if (t.genuine) {
                    genuine_n++;
                    gen_scores.push_back(best);
                    if (best >= o.threshold)
                        genuine_hit++;
                }
                else {
                    impostor_n++;
                    imp_scores.push_back(best);
                    if (best >= o.threshold)
                        impostor_hit++;
                }
            }

            if (keep_trials)
                trials.push_back(std::move(t));
        }
        std::fprintf(stderr, "\r%lld comparisons", cmp_count);
    }
    std::fprintf(stderr, "\n");
    std::fclose(csv);

    /* Pair accounting: what was compared, and what never made it in. */
    std::fprintf(stderr, "fingers            : %d\n", subjects);
    std::fprintf(stderr, "genuine trials     : %ld\n", genuine_n);
    std::fprintf(stderr, "impostor trials    : %ld\n", impostor_n);
    std::fprintf(stderr, "skipped subjects   : %d (reasons on stderr above)\n",
                 skipped);
    if (failed_captures)
        std::fprintf(stderr,
                     "failed captures    : %d probes (%ld trials, excluded "
                     "from the rates)\n",
                     failed_captures, failed_trials);

    if (!o.scores.empty()) {
        std::FILE* f = std::fopen(o.scores.c_str(), "w");

        if (!f) {
            std::fprintf(stderr, "cannot write %s\n", o.scores.c_str());
            return 1;
        }
        std::fprintf(f, "trial_id\tkind\tprobe_finger\tgallery_finger\t"
                        "best_score\tcomparison_scores\n");
        for (const Trial& t : trials) {
            std::fprintf(f, "%ld\t%s\t%s\t%s\t%d\t", t.id,
                         t.genuine ? "genuine" : "impostor",
                         probes[t.probe].subject.c_str(),
                         galleries[t.gallery].front().subject.c_str(), t.best);
            for (size_t i = 0; i < t.cmp.size(); i++)
                std::fprintf(f, "%s%d", i ? " " : "", t.cmp[i]);
            std::fprintf(f, "\n");
        }
        std::fclose(f);
    }

    size_t exported = 0;
    if (!o.export_dir.empty()) {
        /* Tail-biased subset: the trials a replay has to get right are the
         * impostors that scored high, the genuines that scored low, and
         * anything that failed outright. The random tail guards against
         * tuning the replay to the extremes alone. */
        const size_t K = 20, CAP = 120;
        std::vector<long> pick_order;
        std::vector<char> chosen(trials.size(), 0);
        auto take = [&](long id) {
            if (!chosen[id]) {
                chosen[id] = 1;
                pick_order.push_back(id);
            }
        };

        /* Failed captures come first, but bounded like every other category:
         * one blank finger fails every probe against every gallery, and an
         * unbounded set of them would crowd the tails out of the subset and
         * leave the replay with nothing it can actually lock. */
        size_t failures = 0;
        for (const Trial& t : trials) {
            if (!t.failed || failures >= K)
                continue;
            take(t.id);
            failures++;
        }

        /* The score tails are about scores, so a trial that never produced
         * one stays out of them; it is already covered above. */
        std::vector<long> gen_ids, imp_ids;
        for (const Trial& t : trials)
            if (!t.failed)
                (t.genuine ? gen_ids : imp_ids).push_back(t.id);

        auto by_score_desc = [&](long a, long b) {
            if (trials[a].best != trials[b].best)
                return trials[a].best > trials[b].best;
            return a < b;
        };
        auto by_score_asc = [&](long a, long b) {
            if (trials[a].best != trials[b].best)
                return trials[a].best < trials[b].best;
            return a < b;
        };

        std::sort(imp_ids.begin(), imp_ids.end(), by_score_desc);
        for (size_t i = 0; i < imp_ids.size() && i < K; i++)
            take(imp_ids[i]);

        std::sort(gen_ids.begin(), gen_ids.end(), by_score_asc);
        for (size_t i = 0; i < gen_ids.size() && i < K; i++)
            take(gen_ids[i]);
        for (size_t i = 0; i < gen_ids.size() && i < K; i++)
            take(gen_ids[gen_ids.size() - 1 - i]);

        std::vector<long> rest;
        for (const Trial& t : trials)
            if (!chosen[t.id])
                rest.push_back(t.id);

        /* Raw generator output, no library distribution: the draw has to be
         * reproducible from the seed alone. */
        std::mt19937 pick(
            static_cast<uint32_t>(splitmix64(o.seed ^ 0x5eb5ULL)));
        for (size_t i = 0; i < K && !rest.empty(); i++) {
            const size_t j = pick() % rest.size();

            take(rest[j]);
            rest[j] = rest.back();
            rest.pop_back();
        }

        if (pick_order.size() > CAP)
            pick_order.resize(CAP);
        std::sort(pick_order.begin(), pick_order.end());

        if (mkdir(o.export_dir.c_str(), 0755) != 0) {
            struct stat st;

            if (stat(o.export_dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
                std::fprintf(stderr, "cannot create %s\n",
                             o.export_dir.c_str());
                return 1;
            }
        }

        /* Clear what a previous export left: a run with fewer stages, or a
         * different seed, would otherwise leave windows behind that no
         * manifest row references, and a consumer globbing *.pgm would read
         * them as part of this subset. Only this tool's own file names are
         * touched -- whatever else lives in the directory is not ours. */
        if (DIR* d = opendir(o.export_dir.c_str())) {
            std::vector<std::string> stale;

            while (dirent* e = readdir(d))
                if (is_export_artifact(e->d_name))
                    stale.push_back(o.export_dir + "/" + e->d_name);
            closedir(d);

            for (const std::string& p : stale)
                unlink(p.c_str());
        }

        const std::string manifest_path = o.export_dir + "/manifest.tsv";
        std::FILE* mf = std::fopen(manifest_path.c_str(), "w");
        if (!mf) {
            std::fprintf(stderr, "cannot write %s\n", manifest_path.c_str());
            return 1;
        }
        std::fprintf(mf, "trial_id\tkind\tdataset\tprobe_finger\t"
                         "gallery_finger\tstages\tprobe_sha256\t"
                         "gallery_sha256s\texpected_scores\texpected_best\t"
                         "window\tscale\tseed\n");

        /* Reading each source once per selected trial would dominate the
         * export; the subset touches few files. */
        std::map<std::string, Image> src_cache;
        auto source = [&](const std::string& path) -> const Image& {
            auto it = src_cache.find(path);

            if (it == src_cache.end())
                it = src_cache.emplace(path, read_pgm(path)).first;
            return it->second;
        };

        /* Windows are shared between trials (one gallery answers many
         * probes); link the repeats rather than write the same pixels
         * dozens of times. */
        std::map<std::string, std::string> by_hash;
        auto emit = [&](const std::string& name, const Image& win) {
            const std::string hash = sha256_hex(win.px.data(), win.px.size());
            const std::string path = o.export_dir + "/" + name;
            auto it = by_hash.find(hash);

            if (it != by_hash.end()) {
                unlink(path.c_str());
                if (link(it->second.c_str(), path.c_str()) == 0)
                    return hash;
            }
            if (!write_pgm(path, win)) {
                std::fprintf(stderr, "cannot write %s\n", path.c_str());
                std::exit(1);
            }
            if (it == by_hash.end())
                by_hash.emplace(hash, path);
            return hash;
        };

        for (long id : pick_order) {
            const Trial& t = trials[id];
            const Sample& p = probes[t.probe];
            const std::vector<Sample>& gal = galleries[t.gallery];
            char name[64];

            /* Regenerating from the per-trial RNG reproduces the very pixels
             * that were scored; nothing here perturbs the sweep. */
            const Image pwin = window_at(source(p.path), o, p.subject,
                                         ROLE_PROBE, p.index, p.draw);
            if (!pwin.valid()) {
                std::fprintf(stderr, "export: cannot regenerate trial %ld\n",
                             id);
                return 1;
            }
            std::snprintf(name, sizeof(name), "trial%ld_probe.pgm", id);
            const std::string phash = emit(name, pwin);

            std::string ghashes, escores;
            for (size_t s = 0; s < gal.size(); s++) {
                const Image gwin = window_at(source(gal[s].path), o,
                                             gal[s].subject, ROLE_GALLERY,
                                             gal[s].index, gal[s].draw);

                if (!gwin.valid()) {
                    std::fprintf(stderr,
                                 "export: cannot regenerate trial %ld\n", id);
                    return 1;
                }
                std::snprintf(name, sizeof(name), "trial%ld_g%zu.pgm", id, s);
                ghashes += (s ? "," : "") + emit(name, gwin);

                char sc[16];
                std::snprintf(sc, sizeof(sc), "%d",
                              s < t.cmp.size() ? t.cmp[s] : 0);
                escores += (s ? "," : "") + std::string(sc);
            }

            std::fprintf(mf, "%ld\t%s\t%s\t%s\t%s\t%zu\t%s\t%s\t%s\t%d\t"
                             "%dx%d\t%d\t%u\n",
                         t.id, t.genuine ? "genuine" : "impostor",
                         o.dataset_id.c_str(), p.subject.c_str(),
                         gal.front().subject.c_str(), gal.size(),
                         phash.c_str(), ghashes.c_str(), escores.c_str(),
                         t.best, o.width, o.height, o.scale, o.seed);
        }
        std::fclose(mf);
        exported = pick_order.size();
    }

    std::sort(gen_scores.begin(), gen_scores.end());
    std::sort(imp_scores.begin(), imp_scores.end());

    std::printf("matcher=tie+cap176 seed=%u window=%dx%d scale=%d stages=%d "
                "dataset=%s commit=%s\n",
                o.seed, o.width, o.height, o.scale, o.stages,
                o.dataset_id.c_str(), SIGFM_BENCH_COMMIT);
    std::printf("subjects            : %d\n", subjects);
    std::printf("window              : %dx%d (upscaled x%d)\n", o.width,
                o.height, o.scale);
    std::printf("gallery/probes      : %d/%d per subject\n", o.stages,
                o.probes);
    std::printf("score comparisons   : %lld\n", cmp_count);
    std::printf("threshold           : %d\n", o.threshold);
    std::printf("genuine attempts    : %ld\n", genuine_n);
    std::printf("impostor attempts   : %ld\n", impostor_n);
    if (genuine_n)
        std::printf("TAR                 : %.4f  (FRR %.4f)\n",
                    double(genuine_hit) / genuine_n,
                    1.0 - double(genuine_hit) / genuine_n);

    /* No bare point estimate for the false accept rate: it is reported below
     * with its interval and its raw counts, at every operating point. */

    print_percentiles("genuine scores", gen_scores);
    print_percentiles("impostor scores", imp_scores);

    /* Operating points: the proposed threshold between the two the reviewers
     * asked about. Duplicates collapse. */
    std::vector<int> points;
    for (int t : { 24, o.threshold, 1000 })
        if (std::find(points.begin(), points.end(), t) == points.end())
            points.push_back(t);

    for (int t : points) {
        char label[32];
        long hits = 0;

        for (int s : imp_scores)
            if (s >= t)
                hits++;
        std::snprintf(label, sizeof(label), "FAR @ %d", t);
        print_rate(label, hits, impostor_n);
    }
    for (int t : points) {
        char label[32];
        long miss = 0;

        for (int s : gen_scores)
            if (s < t)
                miss++;
        std::snprintf(label, sizeof(label), "FRR @ %d", t);
        print_rate(label, miss, genuine_n);
    }

    /* Supplementary: where the two curves cross. A single number over
     * dependent trials, so it is reported, not headlined. */
    if (!gen_scores.empty() && !imp_scores.empty()) {
        /* Both rates are step functions of the threshold: FAR counts scores
         * at or above it, FRR counts scores below it, so each changes only
         * just past an observed score and both are constant in between. The
         * candidates are therefore 0 and one past every observed score --
         * O(n log n) in the trial count. Walking every integer up to the
         * largest score instead would cost O(max score), and this matcher's
         * scores run to 1e8. */
        std::vector<long> cand;

        cand.reserve(gen_scores.size() + imp_scores.size() + 1);
        cand.push_back(0);
        for (int s : gen_scores)
            cand.push_back(static_cast<long>(s) + 1);
        for (int s : imp_scores)
            cand.push_back(static_cast<long>(s) + 1);
        std::sort(cand.begin(), cand.end());
        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());

        double best_gap = 2.0, eer = 0.0;
        long eer_t = 0;

        for (long t : cand) {
            const double far =
                double(imp_scores.end() -
                       std::lower_bound(imp_scores.begin(), imp_scores.end(),
                                        t)) /
                imp_scores.size();
            const double frr =
                double(std::lower_bound(gen_scores.begin(), gen_scores.end(),
                                        t) -
                       gen_scores.begin()) /
                gen_scores.size();
            const double gap = std::fabs(far - frr);

            if (gap < best_gap) {
                best_gap = gap;
                eer = (far + frr) / 2.0;
                eer_t = t;
            }
        }
        std::printf("EER (approx)        : %.3f%%  @ score %ld\n", eer * 100.0,
                    eer_t);
    }

    std::printf("csv                 : %s\n", o.out.c_str());
    if (!o.scores.empty())
        std::printf("scores              : %s\n", o.scores.c_str());
    if (!o.export_dir.empty())
        std::printf("export              : %s (%zu trials)\n",
                    o.export_dir.c_str(), exported);

    for (auto& gal : galleries)
        for (auto& s : gal)
            sigfm_free_info(s.info);
    for (auto& p : probes)
        sigfm_free_info(p.info);

    return 0;
}
