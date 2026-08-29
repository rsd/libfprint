# SIGFM: matching benchmark

SIGFM matches fingerprints by keypoint description (SIFT features via
OpenCV) instead of the minutiae layout NBIS/Bozorth3 relies on, which is
what lets it work on capture areas too small to reliably locate minutiae.
A SIGFM print stores keypoint descriptor sets rather than NBIS's
`xyt_struct`, and its match score counts mutually consistent keypoint
pairs rather than minutiae pairs, so a SIGFM score is not on the same
scale as a Bozorth3 score and the two must never be compared directly.

This document covers the benchmark harness built on top of the matcher:
`sigfm-bench` (this directory) and `tests/benchmark-matching.py`. It does
not cover the matcher's internals, which are in `sigfm.cpp`/`sigfm.h`.

## Quick start

The benchmark runs against FVC2000, a public fingerprint verification
competition dataset. The commands below use the B-sets (10 fingers, 8
impressions each), which Bologna publishes for download without
registration:

```sh
mkdir -p ~/fvc2000 && cd ~/fvc2000
for db in DB1_B DB2_B DB3_B DB4_B; do
    wget http://bias.csr.unibo.it/fvc2000/Downloads/${db}.zip
done

# Record checksums for your own copies the first time you download them --
# Bologna does not publish official ones. The MR discussion recorded the
# values it saw as a reference point, not a guarantee the files never change.
sha256sum DB*_B.zip
```

Unzip each database into its own directory and convert the TIFF
impressions to the PGM format `sigfm-bench` reads:

```sh
for db in DB1_B DB2_B DB3_B DB4_B; do
    mkdir -p $db && unzip -q -d $db ${db}.zip
    mogrify -format pgm $db/*.tif
done
```

FVC2000 filenames are already `<finger>_<impression>.tif` (`101_1.tif` ...
`110_8.tif`), which `mogrify` carries over as `101_1.pgm` etc. -- exactly
the naming `sigfm-bench` expects (it treats everything before the first
underscore as the finger id), so no renaming step is needed. If a
particular archive unpacks its images into a further subdirectory instead
of `$db` itself, flatten it before converting: `sigfm-bench` hard-errors
on an input directory that hides `.pgm` files below the top level
(`one run, one dataset directory ... also holds .pgm files`), which is
the tell that a flatten step was skipped.

`sigfm-bench` is not part of the default build (`build_by_default: false`
in `libfprint/sigfm/meson.build`, so an ordinary `meson compile` does not
produce it); build it explicitly once:

```sh
meson compile -C builddir sigfm-bench
```

Then run the benchmark on **one** database directory at a time:

```sh
python3 tests/benchmark-matching.py -i ~/fvc2000/DB1_B --builddir builddir
```

This sweeps the dataset with `sigfm-bench`, replays a subset of the trials
through the real `virtual_image` device to prove the sweep and the device
agree, and prints one report: pair counts, TAR/FAR/FRR at the configured
operating points with confidence intervals, score percentiles, an
approximate EER, and a secondary NBIS block on the same crops. Pass
`--quick` for a faster, reduced sweep, or `--help` for the rest of the
options (gallery size, probe count, seed, threshold, export directory,
replay cap).

## What the numbers measure

Label any of this benchmark's numbers as what they are: **system verify,
OR of N enrolled images, synthetic crops of an FVC2000 database**. Never
quote them as an FVC EER.

For each finger, the tool keeps exactly one representative impression (the
first one by filename) and cuts random 112x88 windows out of it -- some
tagged as "gallery" crops standing in for enrollment stages, others tagged
as "probe" crops standing in for a later verify attempt. A genuine trial
compares a probe crop against the gallery crops drawn from the *same*
finger's representative impression; libfprint's own verify logic decides
the trial by checking the probe against each of the N gallery prints in
turn and accepting on the first one that clears the threshold (an OR of
N, not an average or a best-of). An impostor trial compares a probe crop
from one finger against the gallery crops of a different finger.

Because a genuine trial's gallery and probe crops come from one and the
same captured impression, this protocol emulates only the placement and
rotation noise that differs between an enrollment and a later verify
attempt against a fixed source image. It does not carry the noise that
comes from a finger actually being lifted, repositioned and pressed a
second time on a real sensor. That is a deliberate emulation, matching the
same-impression protocol discussed and agreed for this benchmark during
the !530 review, and it is why the numbers are not FVC EER: FVC's own
protocol scores genuine pairs across genuinely separate impressions.

There is no option to draw gallery and probe crops from two different
impressions of the same finger; the tool always uses the one representative
impression described above for both roles.

## Threshold policy

This benchmark reports operating points at chosen thresholds; it does not
set production driver defaults. A production default belongs to the
sensor it ships with, and should come from an impostor tail measured on
that sensor -- the model this project already follows: an Elan driver's
default was raised on the strength of exactly that kind of on-sensor
measurement, not an FVC number. FVC-derived thresholds from this benchmark
argue only a floor for a new driver: at the same 112x88 capture geometry,
the threshold needed to hold a fixed FAR varies roughly 14x across the
sensor technologies represented in the FVC databases, so a threshold that
looks safe on one database can be far too permissive on the sensor a
driver actually ships against.

## A note on FVC2000 A-sets

The quick start above uses the B-sets on purpose. FVC2000's larger
A-sets (100 fingers each) need more care than a drop-in scale-up: the
!530 merge-request discussion documents a case of a same-numbered finger
scoring in the genuine range across two different A-set databases (DB1's
finger 106 against DB2's finger 106) -- a per-database identity
recurrence, not a coincidence. DB1 and DB2 also describe an identical
volunteer pool on FVC2000's own site ("mainly from 20 to 30 year-old
students," four fingers collected per volunteer). That is not established
for every A-set pairing: DB3's own description names a different pool
(19 volunteers aged 5 to 73, six fingers per volunteer), so this note
does not extend the DB1/DB2 case to DB3, and DB4 is synthetic and shares
no identity with any of the others.

Either way, a per-database finger id is just a per-database label:
nothing guarantees finger 47 in one A-set's numbering means anything
relative to finger 47 in another's. Pooling more than one A-set into a
single run's input directory risks turning an actual same-finger pair
into a counted impostor trial (or the reverse) with no way to notice
from the ids alone. The tool structurally avoids this by taking exactly
one dataset directory and one `--dataset-id` per invocation, so run each
database separately. The one hazard it cannot catch on its own is
copying impressions from two different FVC databases into the same
directory by hand -- don't.

## Expected wall time

The committed fixture set (`tests/benchmark-fixtures/`, 4 fingers) is what
`meson test` exercises; it runs in a few seconds and makes no statistical
claim. On a real FVC2000 B-set (10 fingers):

- Full run (`-i <dir> --builddir <builddir>`, both gallery sizes):
  measured on a 32-core benchmark host, 7-59 s depending on the database
  (DB1_B 7 s, DB2_B 28 s, DB3_B 46 s, DB4_B 59 s -- larger, denser images
  cost more SIFT extraction and matching time).
- `--quick` (one gallery size, reduced sweep): measured on a desktop-class
  machine, 4 s (DB1_B).

## Score log and environment variables

These are debug and benchmark facilities read by libfprint and by
`virtual_image`; they are not meant for a shipping driver or device and
are not part of the public API.

- `FP_PRINT_SCORE_LOG=<path>` -- appended to (not truncated) by
  `fpi_print_score_log()` in `fpi-print.c`, right after each comparison it
  scores and before the accept/reject decision on it. One line per
  comparison, space-separated, no header:

  ```
  ALGO INDEX SCORE THRESHOLD
  ```

  `ALGO` is `sigfm` or `nbis`; `INDEX` is the 0-based position of the
  compared print within the template's gallery; `SCORE` and `THRESHOLD`
  are the raw values the comparison was decided on. This is how the
  benchmark tool reads libfprint's internal per-comparison scores without
  a private API.
- `FP_VIRTUAL_IMAGE_ALGORITHM=sigfm|nbis` -- selects the algorithm
  `virtual_image`'s enrolled prints use; read once in `class_init`, so it
  is fixed for the life of the process. `sigfm` requires libfprint to have
  been built with OpenCV (`HAVE_SIGFM`).
- `FP_VIRTUAL_IMAGE_SCORE_THRESHOLD=<n>` -- overrides the running
  process's match threshold. Leaving it unset while `ALGORITHM=sigfm` is
  set does **not** fall back to a SIGFM-scale default: `virtual_image`
  sets no SIGFM-specific default of its own, so the threshold silently
  inherits the NBIS-scale default (`BOZORTH3_DEFAULT_THRESHOLD`, 40) --
  set both variables together.
- `FP_VIRTUAL_IMAGE_ENROLL_STAGES=<n>` -- overrides the number of
  enrollment stages captured (library default 5).
- `FP_VIRTUAL_IMAGE_HOT_SECONDS=<n>` -- temperature-management timing,
  unrelated to matching; the benchmark tool sets it to `-1` to disable
  the feature during automated runs.

## NBIS on full-size images

The benchmark is SIGFM-only: `sigfm-bench` calls `sigfm_extract()` and
`sigfm_match_score()` directly, and `tests/benchmark-matching.py` has no
flag to change the 112x88 crop geometry it asks `sigfm-bench` for. What
the tool does offer is `--smoke <dir>`, which takes any already-exported
directory (a `manifest.tsv` plus the PGM files it names) and runs it
through both the SIGFM lock and an NBIS pass via the same worker
subprocess, reporting NBIS gallery-enrollment and probe-extraction
failure counts on those exact images -- no FAR/FRR/EER, since `--smoke`
never computes rate statistics regardless of what produced the directory.

That is enough to run NBIS against effectively whole images: export at a
window size matching the source images and with no rotation or shift, then
smoke-test that export -- **with one change from the obvious version of
this recipe**, explained below. Reusing the `DB1_B` directory from the
quick start above (already converted to `.pgm`):

```sh
identify -format '%wx%h\n' ~/fvc2000/DB1_B/*.pgm | sort -u   # confirm one size: 300x300

builddir/libfprint/sigfm/sigfm-bench -i ~/fvc2000/DB1_B \
    -W 300 -H 300 --scale 1 --rot 0 --shift 0 --stages 1 --probes 1 \
    --dataset-id DB1_B-fullsize --export-trials /tmp/DB1_B-fullsize

python3 tests/benchmark-matching.py --smoke /tmp/DB1_B-fullsize --builddir builddir
```

`--rot 0 --shift 0` removes the random placement noise that would
otherwise need room to move inside a window the size of the whole image;
`--stages 1 --probes 1` avoids re-exporting the same full image under
several trial names. Verified end to end against real, downloaded
FVC2000 DB1_B data (5 subjects, 25 trials): completes in under 5 seconds
and reports `galleries enrolled : 5 (sigfm), 5 (nbis, 0 failed)` and
`nbis probe failures : 0 genuine, 0 impostor` in the "smoke invariants"
block, which is where to read the NBIS counts this recipe is for.

**`--scale 1` is not optional here, and leaving it at `sigfm-bench`'s
default of 2 does not just run slower -- it hangs.** The worker process
that replays an export sends each image over a Unix domain socket to the
real `virtual_image` device in one blocking write, and only afterwards
starts pumping the event loop that lets the device's read side drain
that socket; if the payload (`width * height * scale^2` pixels, plus a
small PGM header) is larger than the socket's send buffer, the write
blocks forever waiting for a reader that only the code after the write
would start, and the run sits until `tests/benchmark-matching.py`'s own
`WORKER_TIMEOUT` (300 s) kills it with "the worker was killed". This was
reproduced directly: at the default `--scale 2`, `-W 300 -H 300` sends a
600x600 (360,000-byte) image, comfortably past this machine's socket
send buffer (`/proc/sys/net/core/wmem_default`, 212,992 bytes here), and
the stuck worker's kernel stack sat in `sock_alloc_send_pskb` -- a
blocked send, not a slow computation -- for as long as it was left
running. At `--scale 1` the same 300x300 image is a 90,000-byte payload,
well under that budget, and the run completes in seconds. The 112x88
crops this benchmark normally exports (scaled to 224x176, ~39 KB) never
approach this limit, which is why the sizing only bites on a recipe like
this one that deliberately sends whole images.

This means the recipe above is safe for DB1_B (300x300 = 90,000 bytes),
DB2_B (256x364 = 93,184 bytes) and DB4_B (240x320 = 76,800 bytes) at
`--scale 1` on a machine with the same, common 208 KiB default send
buffer. **DB3_B (448x478 = 214,144 bytes) is not**: it is already over
that particular budget even at `--scale 1`, so this recipe was not run
against DB3_B and should not be pointed at it without first checking the
target machine's actual socket-buffer limit (`cat
/proc/sys/net/core/wmem_default`) against the image size in bytes, or
cropping DB3_B to something smaller than its native resolution instead
of trying to run it truly full-size.
