#!/usr/bin/env python3
#
# Copyright (C) 2026 Sergey Subbotin <ssubbotin@gmail.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

"""One command: sweep with sigfm-bench, then replay the same trials through
the real virtual_image device and prove the two paths agree.

    tests/benchmark-matching.py -i <dataset-dir> [--quick] [--full-replay]
    tests/benchmark-matching.py --smoke <fixture-dir>
    tests/benchmark-matching.py --selftest

The sweep (sigfm-bench) provides the statistics; it links SIGFM directly and
never touches a device. The replay provides the validation: every exported
trial is enrolled and verified through libfprint's real image-device pipeline,
with the raw PGM payload bytes pushed over the virtual_image socket, and the
pixels, the per-comparison scores and the accept/reject decisions must agree
with the sweep's own manifest. A disagreement fails the run; the tool never
prints a partial table.

Only the worker half imports gi. The device's algorithm, score threshold and
enrol-stage count are class-level environment configuration read once per
process, so every (algorithm, threshold, stages) combination gets its own
subprocess.
"""

import argparse
import collections
import hashlib
import json
import math
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

# The suite convention: a missing dependency is a skip, not a failure.
EXIT_SKIP = 77

# Unreachable by construction, and the largest value the driver's environment
# parser accepts.  At this threshold nothing matches, so no gallery comparison
# is skipped by an early return and every stage reaches the score log.
LOCK_THRESHOLD = 2147483647

# Wall clock a single device pass may take before it is killed.  Generous
# against a --full-replay of 120 trials on a slow machine, short enough that a
# wedged worker fails a CI job instead of stalling it.
WORKER_TIMEOUT = 300

MANIFEST_COLUMNS = [
    'trial_id', 'kind', 'dataset', 'probe_finger', 'gallery_finger', 'stages',
    'probe_sha256', 'gallery_sha256s', 'expected_scores', 'expected_best',
    'window', 'scale', 'seed',
]

SRC_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Failure(Exception):
    """A condition that must abort the run rather than degrade the report."""


class Skip(Exception):
    """A missing build-time dependency; the caller exits 77."""


# --------------------------------------------------------------------------
# Pure helpers (covered by --selftest)
# --------------------------------------------------------------------------

class Trial:
    """One row of manifest.tsv."""

    __slots__ = tuple(MANIFEST_COLUMNS)

    def __init__(self, **kw):
        for k in MANIFEST_COLUMNS:
            setattr(self, k, kw[k])

    @property
    def genuine(self):
        return self.kind == 'genuine'

    @property
    def probe_file(self):
        return 'trial%d_probe.pgm' % self.trial_id

    @property
    def gallery_files(self):
        return ['trial%d_g%d.pgm' % (self.trial_id, i)
                for i in range(self.stages)]

    @property
    def expected_failure(self):
        """sigfm-bench records a negative score for a failed comparison."""
        return any(s < 0 for s in self.expected_scores)

    def expected_decision(self, threshold):
        return self.expected_best >= threshold


def parse_manifest(path):
    """Read manifest.tsv, binding to the header names rather than to column
    positions."""
    with open(path, 'r') as f:
        rows = [line.rstrip('\n').split('\t') for line in f if line.strip()]

    if not rows:
        raise ValueError('%s: empty manifest' % path)

    header = rows[0]
    missing = [c for c in MANIFEST_COLUMNS if c not in header]
    if missing:
        raise ValueError('%s: manifest is missing column(s) %s'
                         % (path, ', '.join(missing)))
    index = {name: i for i, name in enumerate(header)}

    trials = []
    for lineno, row in enumerate(rows[1:], start=2):
        if len(row) != len(header):
            raise ValueError('%s:%d: %d fields, header has %d'
                             % (path, lineno, len(row), len(header)))

        def cell(name):
            return row[index[name]]

        kind = cell('kind')
        if kind not in ('genuine', 'impostor'):
            raise ValueError('%s:%d: unknown kind %r' % (path, lineno, kind))

        m = re.fullmatch(r'(\d+)x(\d+)', cell('window'))
        if not m:
            raise ValueError('%s:%d: bad window %r'
                             % (path, lineno, cell('window')))

        gallery = cell('gallery_sha256s').split(',')
        scores = [int(s) for s in cell('expected_scores').split(',')]
        stages = int(cell('stages'))
        if len(gallery) != stages or len(scores) != stages:
            raise ValueError(
                '%s:%d: stages=%d but %d gallery hashes and %d scores'
                % (path, lineno, stages, len(gallery), len(scores)))

        trials.append(Trial(
            trial_id=int(cell('trial_id')),
            kind=kind,
            dataset=cell('dataset'),
            probe_finger=cell('probe_finger'),
            gallery_finger=cell('gallery_finger'),
            stages=stages,
            probe_sha256=cell('probe_sha256'),
            gallery_sha256s=gallery,
            expected_scores=scores,
            expected_best=int(cell('expected_best')),
            window=(int(m.group(1)), int(m.group(2))),
            scale=int(cell('scale')),
            seed=int(cell('seed')),
        ))

    if not trials:
        raise ValueError('%s: header only, no trials' % path)
    return trials


def parse_pgm(blob):
    """Parse a binary (P5) PGM and return (width, height, payload).

    Written out by hand so the tool needs neither cairo nor PIL: the driver
    wants the raw payload bytes and nothing else.
    """
    pos = 0
    end = len(blob)

    def token():
        nonlocal pos
        while pos < end:
            c = blob[pos:pos + 1]
            if c == b'#':
                while pos < end and blob[pos:pos + 1] not in (b'\n', b'\r'):
                    pos += 1
            elif c.isspace():
                pos += 1
            else:
                break
        start = pos
        while pos < end:
            c = blob[pos:pos + 1]
            if c.isspace() or c == b'#':
                break
            pos += 1
        if start == pos:
            raise ValueError('truncated PGM header')
        return blob[start:pos]

    magic = token()
    if magic != b'P5':
        raise ValueError('not a binary PGM (magic %r)' % magic)

    try:
        width = int(token())
        height = int(token())
        maxval = int(token())
    except ValueError as e:
        raise ValueError('bad PGM header: %s' % e)

    if width < 1 or height < 1:
        raise ValueError('bad PGM dimensions %dx%d' % (width, height))
    if maxval != 255:
        raise ValueError('PGM maxval %d, expected 255 (8 bits)' % maxval)

    # Exactly one whitespace byte separates the header from the payload; a
    # second one is already a pixel.
    if pos >= end or not blob[pos:pos + 1].isspace():
        raise ValueError('no whitespace after PGM maxval')
    pos += 1

    payload = blob[pos:]
    if len(payload) != width * height:
        raise ValueError('PGM payload is %d bytes, %dx%d needs %d'
                         % (len(payload), width, height, width * height))
    return width, height, payload


def read_pgm(path):
    with open(path, 'rb') as f:
        return parse_pgm(f.read())


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def wilson95(k, n):
    """95% Wilson score interval, clamped to [0, 1].

    Same constant and same arrangement as sigfm-bench's wilson95(), so the two
    halves of the benchmark quote the same numbers.
    """
    z = 1.959963984540054
    if n <= 0:
        return (0.0, 0.0)

    p = float(k) / n
    z2 = z * z
    denom = 1.0 + z2 / n
    centre = (p + z2 / (2.0 * n)) / denom
    half = (z / denom) * math.sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n))
    return (max(0.0, centre - half), min(1.0, centre + half))


def fmt_rate(k, n):
    """A count with its interval, in sigfm-bench's wording.

    A rate of zero is not evidence that the rate is zero, so it prints the
    rule-of-three upper bound instead of a bare 0.
    """
    if n <= 0:
        return '0/0  (no trials)'
    if k == 0:
        return '0/%d  (95%% upper bound %.3f%%)' % (n, min(300.0 / n, 100.0))
    lo, hi = wilson95(k, n)
    return '%d/%d  (95%% CI %.3f%%..%.3f%%)' % (k, n, lo * 100.0, hi * 100.0)


# --------------------------------------------------------------------------
# Worker: the only half that imports gi
# --------------------------------------------------------------------------

def worker_main(job_path, out_path):
    try:
        import gi
        gi.require_version('FPrint', '2.0')
        from gi.repository import GLib, Gio, FPrint
    except Exception as e:
        print('Missing dependencies: %s' % e, file=sys.stderr)
        return EXIT_SKIP

    with open(job_path) as f:
        job = json.load(f)

    loop_ctx = GLib.main_context_default()
    result = {'algorithm': job['algorithm'], 'threshold': job['threshold'],
              'stages': job['stages'], 'trials': [], 'galleries': 0,
              'gallery_failures': 0, 'hashes_checked': 0, 'fatal': None}

    def describe(e):
        # A Failure already reads as a diagnosis; anything else needs its
        # type to be readable at all.
        return str(e) if isinstance(e, Failure) else '%s: %s' % (
            type(e).__name__, e)

    def iterate_until(pred, what, timeout_ms=60000):
        if pred():
            return
        expired = []

        def on_timeout():
            expired.append(True)
            return False

        source = GLib.timeout_add(timeout_ms, on_timeout)
        try:
            while not pred():
                if expired:
                    raise Failure('timed out waiting for %s' % what)
                loop_ctx.iteration(True)
        finally:
            if not expired:
                GLib.source_remove(source)

    try:
        fp_ctx = FPrint.Context()
        dev = None
        for candidate in fp_ctx.get_devices():
            if candidate.get_driver() == 'virtual_image':
                dev = candidate
                break
        if dev is None:
            print('virtual_image driver is not built', file=sys.stderr)
            return EXIT_SKIP

        # Read back what the environment actually configured.  A mismatch
        # here means the library that got loaded is not the one under test
        # (a stale install ahead of it on the search path, say), and every
        # later symptom would be a timeout instead of a diagnosis.
        reported_stages = dev.get_nr_enroll_stages()
        if reported_stages != job['stages']:
            raise Failure(
                'the device reports %d enroll stages after '
                'FP_VIRTUAL_IMAGE_ENROLL_STAGES=%d; the loaded libfprint is '
                'not the build under test'
                % (reported_stages, job['stages']))

        # Load every payload once and check it against the manifest before a
        # single byte reaches the device: a divergence here is "pixels
        # differed", which is a different diagnosis from "matcher differed".
        images = {}
        for name, want in job['payloads'].items():
            width, height, payload = read_pgm(
                os.path.join(job['export_dir'], name))
            if (width, height) != tuple(want['size']):
                raise Failure('%s is %dx%d, the manifest declares %dx%d'
                              % (name, width, height, *want['size']))
            got = sha256_bytes(payload)
            if got != want['sha256']:
                raise Failure('pixels differed: %s payload sha256 %s, '
                              'manifest says %s'
                              % (name, got, want['sha256']))
            images[name] = (width, height, payload)
            result['hashes_checked'] += 1

        dev.open_sync()
        con = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        con.connect(job['socket'])
    except Exception as e:
        result['fatal'] = describe(e)
        with open(out_path, 'w') as f:
            json.dump(result, f)
        return 1

    def send_image(name):
        width, height, payload = images[name]
        con.sendall(struct.pack('ii', width, height) + payload)

    def send_finger_automatic_on():
        con.sendall(struct.pack('ii', -3, 1))

    def wait_for_finger_needed():
        iterate_until(
            lambda: bool(dev.get_finger_status() &
                         FPrint.FingerStatusFlags.NEEDED),
            'the device to ask for a finger')

    def log_tail(offset):
        """The score-log lines appended since `offset`."""
        path = job['score_log']
        if not os.path.exists(path):
            return offset, []
        # Binary, so the offset stays a plain byte count: a text-mode tell()
        # is an opaque cookie and must not be mixed with os.path.getsize().
        with open(path, 'rb') as f:
            f.seek(offset)
            text = f.read().decode('utf-8', 'replace')
            new_offset = f.tell()
        lines = []
        for line in text.splitlines():
            parts = line.split()
            if len(parts) != 4:
                raise Failure('unparsable score-log line %r' % line)
            lines.append([parts[0], int(parts[1]), int(parts[2]),
                          int(parts[3])])
        return new_offset, lines

    def enroll(names):
        """Enrol one gallery through the real multi-stage enroll."""
        state = {'step': 0, 'progress': 0, 'done': False, 'print': None,
                 'error': None}
        cancellable = Gio.Cancellable()

        def progress_cb(*args):
            state['step'] = args[1]
            state['progress'] += 1

        def done_cb(source, res, *rest):
            try:
                state['print'] = source.enroll_finish(res)
            except GLib.Error as e:
                state['error'] = e.message
            state['done'] = True

        template = FPrint.Print.new(dev)
        template.props.finger = FPrint.Finger.RIGHT_INDEX
        template.props.username = 'benchmark-matching'
        dev.enroll(template, cancellable, progress_cb, tuple(), done_cb)

        # A stage whose extraction fails is reported as a retry and does not
        # advance the counter, so the send loop is bounded and gives up
        # rather than spinning.
        budget = 2 * len(names) + 2
        sent = 0
        while (not state['done'] and state['step'] < len(names) and
               sent < budget):
            wait_for_finger_needed()
            before = state['progress']
            send_image(names[state['step']])
            sent += 1
            iterate_until(lambda: state['progress'] > before or state['done'],
                          'enroll stage %d' % state['step'])

        if not state['done'] and state['step'] >= len(names):
            # The last stage was accepted; the completion callback is still
            # in flight behind the progress one.
            iterate_until(lambda: state['done'], 'enroll to complete')

        if not state['done']:
            cancellable.cancel()
            iterate_until(lambda: state['done'], 'enroll to unwind')
            if state['error'] is None:
                state['error'] = ('gave up after %d captures, %d stages '
                                  'accepted' % (sent, state['step']))

        return state['print'], state['error']

    def verify(enrolled, name, offset):
        state = {'done': False, 'match': None, 'error': None}

        def verify_cb(source, res, *rest):
            try:
                state['match'], _ = source.verify_finish(res)
            except GLib.Error as e:
                state['error'] = e.message
            state['done'] = True

        dev.verify(enrolled, callback=verify_cb)
        wait_for_finger_needed()
        send_image(name)
        iterate_until(lambda: state['done'], 'verify to finish')
        offset, lines = log_tail(offset)
        return state, offset, lines

    try:
        # The finger-on/off reports the driver synthesises around each image
        # are what drives the enroll state machine; keep them on.
        send_finger_automatic_on()

        offset = 0
        if os.path.exists(job['score_log']):
            offset = os.path.getsize(job['score_log'])

        # One gallery answers many probes, and identical pixels enrol to an
        # identical template, so key the cache on the manifest hashes.
        cache = {}
        for spec in job['trials']:
            key = tuple(spec['gallery_sha256s'])
            if key not in cache:
                cache[key] = enroll(spec['gallery'])
                result['galleries'] += 1
                if cache[key][0] is None:
                    result['gallery_failures'] += 1
            enrolled, enroll_error = cache[key]

            row = {'trial_id': spec['trial_id'],
                   'enroll_error': enroll_error,
                   'match': None, 'verify_error': None, 'log': []}
            if enrolled is not None:
                state, offset, lines = verify(enrolled, spec['probe'], offset)
                row['match'] = state['match']
                row['verify_error'] = state['error']
                row['log'] = lines
            result['trials'].append(row)
    except Exception as e:
        result['fatal'] = describe(e)
    finally:
        try:
            con.close()
            dev.close_sync()
        except Exception:
            pass

    with open(out_path, 'w') as f:
        json.dump(result, f)
    return 1 if result['fatal'] else 0


# --------------------------------------------------------------------------
# Parent: subprocess plumbing
# --------------------------------------------------------------------------

def have_sigfm(builddir):
    """Whether the build under test compiled SIGFM at all."""
    config = os.path.join(builddir, 'config.h')
    try:
        with open(config) as f:
            return re.search(r'^#define\s+HAVE_SIGFM\b', f.read(),
                             re.MULTILINE) is not None
    except OSError:
        # No config.h to consult; let the device path speak for itself.
        return True


def run_bench(bench, args):
    proc = subprocess.run([bench] + [str(a) for a in args],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True)
    if proc.returncode != 0:
        raise Failure('sigfm-bench %s exited %d\n%s'
                      % (' '.join(str(a) for a in args), proc.returncode,
                         proc.stderr.strip()))
    return proc.stdout


def run_pass(opts, algorithm, threshold, stages, export_dir, trials):
    """One device pass: its own subprocess, its own socket, its own log."""
    # NBIS runs at its own compiled-in threshold; None means "no override",
    # not "threshold unknown", so failure text must not print "None".
    thr = 'n/a' if threshold is None else threshold
    payloads = {}
    specs = []
    for t in trials:
        # The exported window is the upscaled buffer the driver hands to the
        # matcher, so the file must carry window * scale pixels.
        size = [t.window[0] * t.scale, t.window[1] * t.scale]
        payloads[t.probe_file] = {'sha256': t.probe_sha256, 'size': size}
        for name, digest in zip(t.gallery_files, t.gallery_sha256s):
            payloads[name] = {'sha256': digest, 'size': size}
        specs.append({'trial_id': t.trial_id, 'probe': t.probe_file,
                      'gallery': t.gallery_files,
                      'gallery_sha256s': t.gallery_sha256s})

    with tempfile.TemporaryDirectory(prefix='benchmark-matching-') as tmpdir:
        job = {
            'algorithm': algorithm,
            'threshold': threshold,
            'stages': stages,
            'export_dir': os.path.abspath(export_dir),
            'socket': os.path.join(tmpdir, 'bm-%d.sock' % os.getpid()),
            'score_log': os.path.join(tmpdir, 'scores.log'),
            'payloads': payloads,
            'trials': specs,
        }
        job_path = os.path.join(tmpdir, 'job.json')
        out_path = os.path.join(tmpdir, 'result.json')
        with open(job_path, 'w') as f:
            json.dump(job, f)

        libdir = os.path.join(opts.builddir, 'libfprint')
        env = dict(os.environ)
        env['FP_VIRTUAL_IMAGE'] = job['socket']
        env['FP_VIRTUAL_IMAGE_ALGORITHM'] = algorithm
        env['FP_VIRTUAL_IMAGE_ENROLL_STAGES'] = str(stages)
        # Temperature management would throttle a few hundred back-to-back
        # captures; a benchmark harness is not modelling a warm sensor.
        env['FP_VIRTUAL_IMAGE_HOT_SECONDS'] = '-1'
        env['FP_PRINT_SCORE_LOG'] = job['score_log']
        # A caller's shell must not be able to hide the device under test:
        # drop any inherited driver filter before setting our own.  Only the
        # allowlist is read by this libfprint, but a denylist exists in other
        # versions and in fprintd, and inheriting one would surface as the
        # misleading "virtual_image driver is not built" skip.
        for var in ('FP_DRIVERS_ALLOWLIST', 'FP_DRIVERS_DENYLIST'):
            env.pop(var, None)
        env['FP_DRIVERS_ALLOWLIST'] = 'virtual_image'
        env['OPENCV_OPENCL_RUNTIME'] = 'disabled'
        env['OPENCV_FOR_THREADS_NUM'] = '1'
        # NBIS reports a failed minutiae scan with g_warning, which is a
        # legitimate outcome here; the suite's fatal-warnings would abort on
        # it, and the debug spew costs more than it tells us.
        env['G_DEBUG'] = ''
        env['G_MESSAGES_DEBUG'] = ''
        for var in ('LD_LIBRARY_PATH', 'GI_TYPELIB_PATH'):
            env[var] = os.pathsep.join(
                [libdir] + ([env[var]] if env.get(var) else []))
        if threshold is None:
            env.pop('FP_VIRTUAL_IMAGE_SCORE_THRESHOLD', None)
        else:
            env['FP_VIRTUAL_IMAGE_SCORE_THRESHOLD'] = str(threshold)

        try:
            proc = subprocess.run(
                [sys.executable, os.path.abspath(__file__), '--worker',
                 job_path, out_path],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                universal_newlines=True, timeout=WORKER_TIMEOUT)
        except subprocess.TimeoutExpired:
            # Every wait inside the worker is bounded, but a blocked sendall
            # or an unbounded open_sync() is not; without this a wedged
            # worker hangs CI with no diagnostic.
            raise Failure(
                '%s pass (threshold %s, stages %d) did not finish within '
                '%d s; the worker was killed'
                % (algorithm, thr, stages, WORKER_TIMEOUT))

        if proc.returncode == EXIT_SKIP:
            raise Skip(proc.stdout.strip() or proc.stderr.strip())

        result = None
        if os.path.exists(out_path):
            with open(out_path) as f:
                result = json.load(f)

        if proc.returncode != 0 or result is None or result.get('fatal'):
            detail = (result or {}).get('fatal') or 'worker exited %d' % (
                proc.returncode)
            noise = proc.stderr.strip()
            if noise:
                detail += '\n' + indent(noise[-2000:], '  | ')
            raise Failure('%s pass (threshold %s, stages %d) failed: %s'
                          % (algorithm, thr, stages, detail))
        if result['hashes_checked'] != len(payloads):
            raise Failure('%s pass checked %d payloads, %d were sent'
                          % (algorithm, result['hashes_checked'],
                             len(payloads)))
        return result


# --------------------------------------------------------------------------
# Parent: the assertions that make this a lock rather than a demo
# --------------------------------------------------------------------------

class Equivalence:
    def __init__(self):
        self.trials = 0
        self.hashes = 0
        self.comparisons = 0
        self.decisions = 0
        self.locked_kinds = collections.Counter()
        self.skipped = collections.Counter()
        self.problems = []

    @property
    def ok(self):
        return not self.problems

    def note(self, msg):
        self.problems.append(msg)

    def skip(self, reason):
        """A trial that produced no locked comparison, and why."""
        self.skipped[reason] += 1

    def fail(self, reason, msg):
        self.note(msg)
        self.skip(reason)

    def lock(self, trial, comparisons):
        self.trials += 1
        self.comparisons += comparisons
        self.locked_kinds[trial.kind] += 1

    def absorb(self, other):
        self.trials += other.trials
        self.hashes += other.hashes
        self.comparisons += other.comparisons
        self.decisions += other.decisions
        self.locked_kinds += other.locked_kinds
        self.skipped += other.skipped
        self.problems += other.problems

    def skip_summary(self):
        if not self.skipped:
            return '0'
        return '%d (%s)' % (
            sum(self.skipped.values()),
            '; '.join('%s: %d' % (reason, n)
                      for reason, n in sorted(self.skipped.items())))


def check_lock(eq, result, trials, stages, algorithm):
    """Assert pixels, per-comparison scores and best-of-gallery."""
    eq.hashes += result['hashes_checked']
    by_id = {t.trial_id: t for t in trials}

    for row in result['trials']:
        trial = by_id[row['trial_id']]
        tag = 'trial %d (%s)' % (trial.trial_id, trial.kind)

        if row['enroll_error']:
            eq.fail('gallery would not enrol',
                    '%s: gallery enroll failed: %s'
                    % (tag, row['enroll_error']))
            continue
        if row['verify_error']:
            if trial.expected_failure:
                eq.skip('sweep recorded a failed comparison')
                continue
            eq.fail('verify failed',
                    '%s: verify failed: %s' % (tag, row['verify_error']))
            continue
        if row['match']:
            eq.fail('matched at the lock threshold',
                    '%s: matched at the lock threshold, which is unreachable'
                    % tag)
            continue

        log = row['log']
        algos = {line[0] for line in log}
        if algos != {algorithm}:
            eq.fail('wrong algorithm ran',
                    '%s: score log says algorithm %s, the pass asked for %s'
                    % (tag, sorted(algos) or ['nothing'], algorithm))
            continue
        if {line[3] for line in log} != {LOCK_THRESHOLD}:
            eq.fail('wrong threshold ran',
                    '%s: score log threshold %s, expected %d'
                    % (tag, sorted({line[3] for line in log}),
                       LOCK_THRESHOLD))
            continue
        if len(log) != stages:
            eq.fail('gallery cardinality',
                    '%s: %d gallery comparisons logged, gallery holds %d'
                    % (tag, len(log), stages))
            continue
        if [line[1] for line in log] != list(range(stages)):
            eq.fail('gallery indices out of order',
                    '%s: gallery indices %s out of order'
                    % (tag, [line[1] for line in log]))
            continue

        scores = [line[2] for line in log]
        if trial.expected_failure:
            # The sweep marked this trial a failed capture, yet the device
            # produced a full, well-formed score log for it -- identical
            # pixels give identical keypoint counts, so this direction is
            # reachable only when the two MIN_KEYPOINTS copies (fp-image.c
            # and sigfm-bench.cpp) have drifted apart.  Unlike the symmetric
            # verify_error case above, this is not a consistent skip.
            eq.fail('quality gate diverged',
                    '%s: sweep marked trial failed but device scored it '
                    '-- quality-gate mirror drift' % tag)
            continue
        if scores != trial.expected_scores:
            eq.fail('matcher differed',
                    '%s: matcher differed: device scores %s, sweep %s'
                    % (tag, scores, trial.expected_scores))
            continue
        if max(scores) != trial.expected_best:
            eq.fail('best-of-gallery differed',
                    '%s: best-of-gallery %d, manifest says %d'
                    % (tag, max(scores), trial.expected_best))
            continue

        eq.lock(trial, len(log))


def require_lock(eq, trials, tag):
    """A green verdict has to be earned.

    Skipping every trial leaves the problem list empty, which used to read as
    a pass: a build whose SIGFM extraction is broken fails every sweep
    comparison, marks every manifest row as a failure, and would then sail
    through the very check meant to catch it.  An empty lock is a failure.
    """
    if not trials:
        return

    if eq.trials == 0 or eq.comparisons == 0:
        eq.note('%s: nothing was locked (0 of %d replayed trials produced a '
                'comparison; skipped %s)'
                % (tag, len(trials), eq.skip_summary()))
        return

    for kind in ('genuine', 'impostor'):
        present = sum(1 for t in trials if t.kind == kind)
        if present and not eq.locked_kinds[kind]:
            eq.note('%s: no %s trial was locked, though %d were replayed'
                    % (tag, kind, present))


def check_decision(eq, result, trials, threshold, algorithm):
    by_id = {t.trial_id: t for t in trials}
    for row in result['trials']:
        trial = by_id[row['trial_id']]
        tag = 'trial %d (%s)' % (trial.trial_id, trial.kind)
        if trial.expected_failure:
            continue
        if row['enroll_error'] or row['verify_error']:
            eq.note('%s: decision pass could not run: %s'
                    % (tag, row['enroll_error'] or row['verify_error']))
            continue
        # An early return on a match can cut the log short, so only the
        # identity of what ran is checked here, not the line count.
        wrong = {line[0] for line in row['log']} - {algorithm}
        if wrong or not row['log']:
            eq.note('%s: decision pass logged %s, expected %s'
                    % (tag, sorted(wrong) or 'no comparison', algorithm))
            continue
        if {line[3] for line in row['log']} != {threshold}:
            eq.note('%s: decision pass ran at threshold %s, asked for %d'
                    % (tag, sorted({line[3] for line in row['log']}),
                       threshold))
            continue
        want = trial.expected_decision(threshold)
        if bool(row['match']) != want:
            eq.note('%s: decision at %d was %s, manifest best %d says %s'
                    % (tag, threshold, bool(row['match']),
                       trial.expected_best, want))
            continue
        eq.decisions += 1


def summarize_nbis(result, trials):
    """FTE first, then decisions.  There is no NBIS score oracle."""
    by_id = {t.trial_id: t for t in trials}
    out = {'galleries': result['galleries'],
           'gallery_failures': result['gallery_failures'],
           'threshold': None,
           'genuine': 0, 'genuine_fte': 0, 'genuine_match': 0,
           'impostor': 0, 'impostor_fte': 0, 'impostor_match': 0,
           'unusable': 0}

    for row in result['trials']:
        trial = by_id[row['trial_id']]
        if row['enroll_error']:
            out['unusable'] += 1
            continue
        for line in row['log']:
            out['threshold'] = line[3]
        if trial.genuine:
            out['genuine'] += 1
            if row['verify_error']:
                out['genuine_fte'] += 1
            elif row['match']:
                out['genuine_match'] += 1
        else:
            out['impostor'] += 1
            if row['verify_error']:
                out['impostor_fte'] += 1
            elif row['match']:
                out['impostor_match'] += 1
    return out


# --------------------------------------------------------------------------
# Parent: replay subset selection
# --------------------------------------------------------------------------

def replay_subset(trials, cap):
    """Tail-biased pick, mirroring the export's own priority order.

    sigfm-bench already exported a tail-biased subset; this second cap keeps a
    device pass bounded on a large dataset without silently dropping the
    interesting end of either distribution.  --full-replay removes it.
    """
    if cap is None or len(trials) <= cap:
        return list(trials)

    genuine_low = sorted((t for t in trials if t.genuine),
                         key=lambda t: (t.expected_best, t.trial_id))
    impostor_high = sorted((t for t in trials if not t.genuine),
                           key=lambda t: (-t.expected_best, t.trial_id))
    failures = [t for t in trials if t.expected_failure]

    picked, seen = [], set()

    def take(sequence, quota):
        taken = 0
        for t in sequence:
            if len(picked) >= cap or taken >= quota:
                return
            if t.trial_id in seen:
                continue
            seen.add(t.trial_id)
            picked.append(t)
            taken += 1

    # Every failure first, then an equal share of each tail, then whatever
    # else fits.  A straight priority walk would spend the whole cap on the
    # first group.
    take(failures, cap)
    quota = max(1, (cap - len(picked)) // 3)
    take(impostor_high, quota)
    take(genuine_low, quota)
    take(reversed(genuine_low), quota)
    take(trials, cap)
    return sorted(picked, key=lambda t: t.trial_id)


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------

def indent(text, prefix='    '):
    return '\n'.join(prefix + line if line else line
                     for line in text.rstrip('\n').split('\n'))


def section(title):
    return '%s\n%s' % (title, '-' * len(title))


def strip_dead_paths(summary):
    """Drop the sweep's own file listing when those files are about to go.

    Without --export-dir the export lives in a temporary directory that is
    removed before the report is printed, so quoting `csv:`, `scores:` and
    `export:` would advertise paths the reader cannot open.
    """
    return '\n'.join(line for line in summary.rstrip('\n').split('\n')
                     if not re.match(r'(csv|scores|export)\s+:', line))


def opencv_version():
    try:
        proc = subprocess.run(['pkg-config', '--modversion', 'opencv4'],
                              stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL,
                              universal_newlines=True)
        if proc.returncode == 0:
            return proc.stdout.strip()
    except OSError:
        pass
    return 'unknown'


def libfprint_commit():
    try:
        proc = subprocess.run(['git', '-C', SRC_ROOT, 'rev-parse', '--short',
                               'HEAD'], stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL,
                              universal_newlines=True)
        if proc.returncode == 0:
            return proc.stdout.strip()
    except OSError:
        pass
    return 'unknown'


def verdict_line(eq):
    if eq.ok:
        return ('equivalence: OK (%d trials, pixels and scores identical)'
                % eq.trials)
    return ('equivalence: FAILED (%d problem(s) over %d verified trials)'
            % (len(eq.problems), eq.trials))


# --------------------------------------------------------------------------
# Modes
# --------------------------------------------------------------------------

def prepare(opts):
    """Locate the build under test; raise Skip when it cannot be used."""
    opts.builddir = os.path.abspath(opts.builddir)
    if not os.path.isdir(opts.builddir):
        raise Failure('no such build directory: %s' % opts.builddir)
    if not have_sigfm(opts.builddir):
        raise Skip('%s was built without SIGFM (no OpenCV)' % opts.builddir)
    if opts.bench is None:
        opts.bench = os.path.join(opts.builddir, 'libfprint', 'sigfm',
                                  'sigfm-bench')
    opts.bench = os.path.abspath(opts.bench)


def mode_smoke(opts):
    """Pipeline invariants on a pre-exported fixture: no sweep, no rates."""
    prepare(opts)
    manifest = os.path.join(opts.smoke, 'manifest.tsv')
    if not os.path.exists(manifest):
        raise Failure('%s: no manifest.tsv (not an export directory)'
                      % opts.smoke)
    trials = parse_manifest(manifest)
    stages = {t.stages for t in trials}
    if len(stages) != 1:
        raise Failure('fixture mixes gallery sizes %s' % sorted(stages))
    stages = stages.pop()

    eq = Equivalence()
    lock = run_pass(opts, 'sigfm', LOCK_THRESHOLD, stages, opts.smoke, trials)
    check_lock(eq, lock, trials, stages, 'sigfm')
    decision = run_pass(opts, 'sigfm', opts.threshold, stages, opts.smoke,
                        trials)
    check_decision(eq, decision, trials, opts.threshold, 'sigfm')
    require_lock(eq, trials, 'smoke')
    nbis = run_pass(opts, 'nbis', None, stages, opts.smoke, trials)
    nbis_summary = summarize_nbis(nbis, trials)

    out = [verdict_line(eq), '']
    for problem in eq.problems:
        out.append('  ! %s' % problem)
    if eq.problems:
        out.append('')
    out.append(section('smoke invariants'))
    out.append('  fixture              : %s' % os.path.abspath(opts.smoke))
    out.append('  trials               : %d (gallery %d)'
               % (len(trials), stages))
    out.append('  trials locked        : %d of %d (%d genuine, %d impostor)'
               % (eq.trials, len(trials), eq.locked_kinds['genuine'],
                  eq.locked_kinds['impostor']))
    out.append('  trials skipped       : %s' % eq.skip_summary())
    out.append('  payload hashes       : %d identical' % eq.hashes)
    out.append('  score comparisons    : %d identical' % eq.comparisons)
    out.append('  gallery cardinality  : %d comparisons logged per locked '
               'verify' % (eq.comparisons // eq.trials if eq.trials else 0))
    out.append('  decisions @ %-8d : %d agree with the manifest'
               % (opts.threshold, eq.decisions))
    out.append('  galleries enrolled   : %d (sigfm), %d (nbis, %d failed)'
               % (lock['galleries'], nbis['galleries'],
                  nbis['gallery_failures']))
    out.append('  nbis probe failures  : %d genuine, %d impostor'
               % (nbis_summary['genuine_fte'], nbis_summary['impostor_fte']))
    out.append('')
    out.append('no FAR/FRR/EER is reported here: the fixture is plumbing, '
               'not a dataset.')
    print('\n'.join(out))
    return 0 if eq.ok else 1


def sweep_round(opts, stages, export_dir, tag):
    """One sweep plus its lock and decision replays."""
    args = ['-i', opts.input, '--stages', stages, '--probes', opts.probes,
            '--seed', opts.seed, '--threshold', opts.threshold,
            '--dataset-id', opts.dataset_id, '--export-trials', export_dir,
            '-o', os.path.join(export_dir, 'sweep.csv'),
            '--scores', os.path.join(export_dir, 'scores.tsv')]
    if opts.subjects:
        args += ['--subjects', opts.subjects]
    summary = run_bench(opts.bench, args)

    trials = parse_manifest(os.path.join(export_dir, 'manifest.tsv'))
    sizes = {t.stages for t in trials}
    if sizes != {stages}:
        raise Failure('%s: manifest gallery size %s, asked for %d'
                      % (tag, sorted(sizes), stages))
    subset = replay_subset(trials,
                           None if opts.full_replay else opts.replay_max)

    eq = Equivalence()
    lock = run_pass(opts, 'sigfm', LOCK_THRESHOLD, stages, export_dir, subset)
    check_lock(eq, lock, subset, stages, 'sigfm')
    decision = run_pass(opts, 'sigfm', opts.threshold, stages, export_dir,
                        subset)
    check_decision(eq, decision, subset, opts.threshold, 'sigfm')
    require_lock(eq, subset, tag)
    return {'stages': stages, 'summary': summary, 'trials': trials,
            'subset': subset, 'eq': eq, 'dir': export_dir}


def mode_full(opts):
    prepare(opts)
    started = time.time()
    if opts.dataset_id is None:
        opts.dataset_id = os.path.basename(os.path.abspath(opts.input))

    keep = opts.export_dir is not None
    root = (os.path.abspath(opts.export_dir) if keep
            else tempfile.mkdtemp(prefix='benchmark-matching-export-'))
    sizes = [opts.stages]
    if not opts.quick and opts.stages2 != opts.stages:
        sizes.append(opts.stages2)

    rounds = []
    done = False
    try:
        for stages in sizes:
            export_dir = os.path.join(root, 'n%d' % stages)
            os.makedirs(export_dir, exist_ok=True)
            rounds.append(sweep_round(opts, stages, export_dir,
                                      'N=%d' % stages))

        # NBIS runs on the first round's subset only: it is a secondary
        # integration block, and there is no NBIS score oracle to lock it
        # against.
        nbis = run_pass(opts, 'nbis', None, rounds[0]['stages'],
                        rounds[0]['dir'], rounds[0]['subset'])
        nbis_summary = summarize_nbis(nbis, rounds[0]['subset'])
        done = True
    finally:
        if not keep and done:
            shutil.rmtree(root, ignore_errors=True)
        elif not keep:
            # A run that blew up is exactly when the windows and the manifest
            # are worth looking at.
            print('benchmark-matching: exports left at %s' % root,
                  file=sys.stderr)

    total = Equivalence()
    for r in rounds:
        total.absorb(r['eq'])

    out = [verdict_line(total)]
    for r in rounds:
        eq = r['eq']
        out.append('  N=%-3d %4d locked of %4d replayed (%4d exported), '
                   '%5d hashes, %5d comparisons, %4d decisions @ %d'
                   % (r['stages'], eq.trials, len(r['subset']),
                      len(r['trials']), eq.hashes, eq.comparisons,
                      eq.decisions, opts.threshold))
        out.append('        %d genuine, %d impostor locked; skipped %s'
                   % (eq.locked_kinds['genuine'], eq.locked_kinds['impostor'],
                      eq.skip_summary()))
    for problem in total.problems:
        out.append('  ! %s' % problem)
    out.append('')

    for r in rounds:
        out.append(section('sweep, gallery N=%d' % r['stages']))
        out.append(indent(strip_dead_paths(r['summary']) if not keep
                          else r['summary']))
        if not keep:
            out.append('    (sweep artifacts discarded; pass --export-dir to '
                       'keep them)')
        out.append('')
    if opts.quick:
        out.append(section('sweep, gallery N=%d' % opts.stages2))
        out.append('    skipped: --quick runs one gallery size')
        out.append('')

    out.append(section('NBIS on the same crops (secondary, no score oracle)'))
    out.append('    policy: a genuine extraction failure counts as a false')
    out.append('    reject, an impostor extraction failure as a correct')
    out.append('    reject.  No FAR/FRR co-headline: these are integration')
    out.append('    counts, not an operating point.')
    out.append('    threshold           : %s'
               % (nbis_summary['threshold'] if nbis_summary['threshold']
                  is not None else 'never reached (no comparison ran)'))
    out.append('    gallery enrolments  : %d attempted, %d failed'
               % (nbis_summary['galleries'], nbis_summary['gallery_failures']))
    out.append('    trials unusable     : %d (gallery would not enrol)'
               % nbis_summary['unusable'])
    out.append('    genuine probes      : %d' % nbis_summary['genuine'])
    out.append('      extraction failed : %s'
               % fmt_rate(nbis_summary['genuine_fte'],
                          nbis_summary['genuine']))
    out.append('      accepted          : %d' % nbis_summary['genuine_match'])
    out.append('    impostor probes     : %d' % nbis_summary['impostor'])
    out.append('      extraction failed : %s'
               % fmt_rate(nbis_summary['impostor_fte'],
                          nbis_summary['impostor']))
    out.append('      accepted          : %s'
               % fmt_rate(nbis_summary['impostor_match'],
                          nbis_summary['impostor']))
    out.append('')

    banner = re.search(r'matcher=(\S+)', rounds[0]['summary'])
    out.append(section('stamp'))
    out.append('    matcher             : %s'
               % (banner.group(1) if banner else 'unknown'))
    out.append('    libfprint commit    : %s' % libfprint_commit())
    out.append('    opencv              : %s' % opencv_version())
    out.append('    build directory     : %s' % os.path.abspath(opts.builddir))
    out.append('    sigfm-bench         : %s' % opts.bench)
    out.append('    dataset             : %s (%s)'
               % (opts.dataset_id, os.path.abspath(opts.input)))
    out.append('    seed                : %d' % opts.seed)
    out.append('    gallery sizes       : %s'
               % ', '.join(str(r['stages']) for r in rounds))
    out.append('    probes per subject  : %d' % opts.probes)
    out.append('    subjects            : %s' % (opts.subjects or 'all'))
    out.append('    operating threshold : %d' % opts.threshold)
    out.append('    replay cap          : %s'
               % ('none (--full-replay)' if opts.full_replay
                  else opts.replay_max))
    out.append('    exports             : %s'
               % (root if keep else 'discarded (pass --export-dir to keep)'))
    out.append('    python              : %s' % sys.version.split()[0])
    out.append('    wall time           : %.1f s' % (time.time() - started))
    print('\n'.join(out))
    return 0 if total.ok else 1


# --------------------------------------------------------------------------
# Unit cases (--selftest)
# --------------------------------------------------------------------------

class TestManifest(unittest.TestCase):

    MANIFEST = (
        "trial_id\tkind\tdataset\tprobe_finger\tgallery_finger\tstages\t"
        "probe_sha256\tgallery_sha256s\texpected_scores\texpected_best\t"
        "window\tscale\tseed\n"
        "0\tgenuine\tfixture\tarch\tarch\t3\taa\tb1,b2,b3\t10,20,30\t30\t"
        "112x88\t2\t12345\n"
        "1\timpostor\tfixture\tarch\tloopright\t3\taa\tc1,c2,c3\t0,0,0\t0\t"
        "112x88\t2\t12345\n"
    )

    def write(self, text):
        fd, path = tempfile.mkstemp(suffix='.tsv')
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        self.addCleanup(os.unlink, path)
        return path

    def test_roundtrip(self):
        trials = parse_manifest(self.write(self.MANIFEST))
        self.assertEqual(len(trials), 2)
        t = trials[0]
        self.assertEqual(t.trial_id, 0)
        self.assertEqual(t.kind, 'genuine')
        self.assertTrue(t.genuine)
        self.assertEqual(t.gallery_sha256s, ['b1', 'b2', 'b3'])
        self.assertEqual(t.expected_scores, [10, 20, 30])
        self.assertEqual(t.expected_best, 30)
        self.assertEqual(t.stages, 3)
        self.assertEqual(t.window, (112, 88))
        self.assertEqual(t.scale, 2)
        self.assertEqual(t.seed, 12345)
        self.assertFalse(trials[1].genuine)

    def test_derived_filenames(self):
        t = parse_manifest(self.write(self.MANIFEST))[1]
        self.assertEqual(t.probe_file, 'trial1_probe.pgm')
        self.assertEqual(t.gallery_files,
                         ['trial1_g0.pgm', 'trial1_g1.pgm', 'trial1_g2.pgm'])

    def test_expected_decision(self):
        genuine, impostor = parse_manifest(self.write(self.MANIFEST))
        self.assertTrue(genuine.expected_decision(30))
        self.assertFalse(genuine.expected_decision(31))
        self.assertFalse(impostor.expected_decision(1))
        self.assertFalse(genuine.expected_decision(LOCK_THRESHOLD))

    def test_expected_failure_flag(self):
        text = self.MANIFEST.replace('\t10,20,30\t', '\t10,-1,30\t')
        self.assertTrue(parse_manifest(self.write(text))[0].expected_failure)
        self.assertFalse(parse_manifest(self.write(self.MANIFEST))[0]
                         .expected_failure)

    def test_binds_to_header_not_position(self):
        rows = self.MANIFEST.strip().split('\n')
        order = list(reversed(range(len(rows[0].split('\t')))))
        shuffled = '\n'.join('\t'.join(row.split('\t')[i] for i in order)
                             for row in rows) + '\n'
        trials = parse_manifest(self.write(shuffled))
        self.assertEqual(trials[0].expected_scores, [10, 20, 30])
        self.assertEqual(trials[0].probe_finger, 'arch')
        self.assertEqual(trials[0].gallery_finger, 'arch')

    def test_missing_column_rejected(self):
        text = self.MANIFEST.replace('expected_best\t', '')
        with self.assertRaises(ValueError):
            parse_manifest(self.write(text))

    def test_cardinality_mismatch_rejected(self):
        text = self.MANIFEST.replace('\tb1,b2,b3\t', '\tb1,b2\t')
        with self.assertRaises(ValueError):
            parse_manifest(self.write(text))

    def test_header_only_rejected(self):
        with self.assertRaises(ValueError):
            parse_manifest(self.write(self.MANIFEST.split('\n')[0] + '\n'))

    def test_unknown_kind_rejected(self):
        text = self.MANIFEST.replace('\tgenuine\t', '\tmaybe\t')
        with self.assertRaises(ValueError):
            parse_manifest(self.write(text))


class TestPgm(unittest.TestCase):

    def test_minimal(self):
        w, h, px = parse_pgm(b'P5\n2 2\n255\n\x00\x01\x02\x03')
        self.assertEqual((w, h), (2, 2))
        self.assertEqual(px, b'\x00\x01\x02\x03')

    def test_comments_and_spacing(self):
        blob = (b'P5 # first\n# a whole comment line\n 2  2 \n255 '
                b'\xff\xfe\xfd\xfc')
        w, h, px = parse_pgm(blob)
        self.assertEqual((w, h), (2, 2))
        self.assertEqual(px, b'\xff\xfe\xfd\xfc')

    def test_single_whitespace_after_maxval(self):
        # Exactly one whitespace byte terminates the header; a second one is
        # already a pixel.
        w, h, px = parse_pgm(b'P5\n1 2\n255\n\n\x20')
        self.assertEqual((w, h), (1, 2))
        self.assertEqual(px, b'\n\x20')

    def test_rejects_plain_pgm(self):
        with self.assertRaises(ValueError):
            parse_pgm(b'P2\n2 2\n255\n0 1 2 3\n')

    def test_rejects_wrong_maxval(self):
        with self.assertRaises(ValueError):
            parse_pgm(b'P5\n2 2\n65535\n\x00\x01\x02\x03')

    def test_rejects_short_payload(self):
        with self.assertRaises(ValueError):
            parse_pgm(b'P5\n2 2\n255\n\x00\x01\x02')

    def test_rejects_trailing_garbage(self):
        with self.assertRaises(ValueError):
            parse_pgm(b'P5\n2 2\n255\n\x00\x01\x02\x03junk')

    def test_rejects_truncated_header(self):
        with self.assertRaises(ValueError):
            parse_pgm(b'P5\n2 2\n')

    def test_roundtrip_through_a_file(self):
        payload = bytes(range(256)) * 4
        blob = b'P5\n64 16\n255\n' + payload
        fd, path = tempfile.mkstemp(suffix='.pgm')
        with os.fdopen(fd, 'wb') as f:
            f.write(blob)
        self.addCleanup(os.unlink, path)
        w, h, px = read_pgm(path)
        self.assertEqual((w, h, px), (64, 16, payload))
        self.assertEqual(sha256_bytes(px), hashlib.sha256(payload).hexdigest())


class TestWilson(unittest.TestCase):

    # Both reference values are the ones sigfm-bench prints for the same
    # counts (task 6 report, "Statistics, checked against an independent
    # implementation").
    def test_three_in_eighteen_thousand(self):
        lo, hi = wilson95(3, 18000)
        self.assertEqual('%.3f%%..%.3f%%' % (lo * 100, hi * 100),
                         '0.006%..0.049%')

    def test_all_of_thirty_six(self):
        lo, hi = wilson95(36, 36)
        self.assertEqual('%.3f%%..%.3f%%' % (lo * 100, hi * 100),
                         '90.358%..100.000%')

    def test_clamped_to_unit_interval(self):
        lo, hi = wilson95(0, 1)
        self.assertGreaterEqual(lo, 0.0)
        self.assertLessEqual(hi, 1.0)

    def test_no_trials(self):
        self.assertEqual(wilson95(0, 0), (0.0, 0.0))

    def test_rate_line_matches_bench_wording(self):
        self.assertEqual(fmt_rate(0, 36), '0/36  (95% upper bound 8.333%)')
        self.assertEqual(fmt_rate(0, 1), '0/1  (95% upper bound 100.000%)')
        self.assertEqual(fmt_rate(36, 36), '36/36  (95% CI 90.358%..100.000%)')
        self.assertEqual(fmt_rate(0, 0), '0/0  (no trials)')


class TestVerdict(unittest.TestCase):
    """The zero-lock guard: a green verdict has to be earned."""

    MANIFEST_HEAD = (
        "trial_id\tkind\tdataset\tprobe_finger\tgallery_finger\tstages\t"
        "probe_sha256\tgallery_sha256s\texpected_scores\texpected_best\t"
        "window\tscale\tseed\n")

    def manifest(self, failed):
        """A 2-row manifest; `failed` marks every comparison as a sweep
        failure, which is what a build with broken extraction produces."""
        scores = '-1,-1' if failed else '10,20'
        text = self.MANIFEST_HEAD
        for i, kind in enumerate(('genuine', 'impostor')):
            text += ('%d\t%s\tfx\ta\ta\t2\tp\tg1,g2\t%s\t20\t8x8\t1\t1\n'
                     % (i, kind, scores))
        fd, path = tempfile.mkstemp(suffix='.tsv')
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        self.addCleanup(os.unlink, path)
        return parse_manifest(path)

    def test_all_expected_failure_is_not_ok(self):
        trials = self.manifest(failed=True)
        self.assertTrue(all(t.expected_failure for t in trials))
        eq = Equivalence()
        for _ in trials:
            eq.skip('sweep recorded a failed comparison')
        self.assertTrue(eq.ok)          # the hole this guard closes
        require_lock(eq, trials, 'smoke')
        self.assertFalse(eq.ok)
        self.assertIn('nothing was locked', eq.problems[0])
        self.assertIn('sweep recorded a failed comparison: 2', eq.problems[0])
        self.assertIn('FAILED', verdict_line(eq))

    def test_locked_run_is_ok(self):
        trials = self.manifest(failed=False)
        eq = Equivalence()
        for t in trials:
            eq.lock(t, 2)
        require_lock(eq, trials, 'smoke')
        self.assertTrue(eq.ok)
        self.assertIn('equivalence: OK', verdict_line(eq))

    def test_one_sided_lock_is_not_ok(self):
        trials = self.manifest(failed=False)
        eq = Equivalence()
        eq.lock(trials[0], 2)           # genuine only
        eq.skip('verify failed')
        require_lock(eq, trials, 'smoke')
        self.assertFalse(eq.ok)
        self.assertIn('no impostor trial was locked', eq.problems[0])

    def test_empty_subset_is_not_flagged(self):
        eq = Equivalence()
        require_lock(eq, [], 'smoke')
        self.assertTrue(eq.ok)

    def test_skip_summary_wording(self):
        eq = Equivalence()
        self.assertEqual(eq.skip_summary(), '0')
        eq.skip('verify failed')
        eq.skip('verify failed')
        eq.skip('matcher differed')
        self.assertEqual(eq.skip_summary(),
                         '3 (matcher differed: 1; verify failed: 2)')

    def test_absorb_merges_counters(self):
        trials = self.manifest(failed=False)
        a, b = Equivalence(), Equivalence()
        a.lock(trials[0], 2)
        a.skip('verify failed')
        b.lock(trials[1], 2)
        b.note('boom')
        a.absorb(b)
        self.assertEqual(a.trials, 2)
        self.assertEqual(a.comparisons, 4)
        self.assertEqual(a.locked_kinds['genuine'], 1)
        self.assertEqual(a.locked_kinds['impostor'], 1)
        self.assertEqual(a.skipped['verify failed'], 1)
        self.assertFalse(a.ok)


class TestCheckLock(unittest.TestCase):
    """The lock direction that a MIN_KEYPOINTS mirror drift would produce."""

    def trial(self, expected_failure):
        scores = [-1] if expected_failure else [10]
        return Trial(
            trial_id=0, kind='genuine', dataset='d', probe_finger='f',
            gallery_finger='f', stages=1, probe_sha256='p',
            gallery_sha256s=['g'], expected_scores=scores,
            expected_best=scores[0], window=(1, 1), scale=1, seed=1)

    def test_device_scoring_an_expected_failure_fails(self):
        """Sweep predicted a failed capture, but the device scored it
        anyway: that asymmetry is exactly what a MIN_KEYPOINTS mirror drift
        would produce, so the lock must fail rather than skip."""
        trial = self.trial(expected_failure=True)
        result = {'hashes_checked': 0,
                  'trials': [{'trial_id': 0, 'enroll_error': None,
                             'verify_error': None, 'match': False,
                             'log': [['sigfm', 0, 10, LOCK_THRESHOLD]]}]}
        eq = Equivalence()
        check_lock(eq, result, [trial], stages=1, algorithm='sigfm')
        self.assertFalse(eq.ok)
        self.assertIn('quality-gate mirror drift', eq.problems[0])
        self.assertEqual(eq.skipped['quality gate diverged'], 1)

    def test_device_failing_an_expected_failure_still_skips(self):
        """The other direction (sweep-scored but device-failed, or both
        agreeing on a failure) stays a consistent skip, not a lock
        failure."""
        trial = self.trial(expected_failure=True)
        result = {'hashes_checked': 0,
                  'trials': [{'trial_id': 0, 'enroll_error': None,
                             'verify_error': 'no keypoints found',
                             'match': None, 'log': []}]}
        eq = Equivalence()
        check_lock(eq, result, [trial], stages=1, algorithm='sigfm')
        self.assertTrue(eq.ok)
        self.assertEqual(eq.skipped['sweep recorded a failed comparison'], 1)


class TestStripDeadPaths(unittest.TestCase):

    def test_drops_only_the_path_lines(self):
        summary = ('subjects            : 4\n'
                   'EER (approx)        : 0.000%  @ score 1\n'
                   'csv                 : /tmp/gone/sweep.csv\n'
                   'scores              : /tmp/gone/scores.tsv\n'
                   'export              : /tmp/gone (48 trials)\n')
        kept = strip_dead_paths(summary).split('\n')
        self.assertEqual(len(kept), 2)
        self.assertTrue(kept[0].startswith('subjects'))
        self.assertTrue(kept[1].startswith('EER'))


class TestSubset(unittest.TestCase):

    def make(self, n):
        trials = []
        for i in range(n):
            genuine = i % 2 == 0
            trials.append(Trial(
                trial_id=i, kind='genuine' if genuine else 'impostor',
                dataset='d', probe_finger='f', gallery_finger='f',
                stages=1, probe_sha256='p', gallery_sha256s=['g'],
                expected_scores=[i], expected_best=i, window=(1, 1),
                scale=1, seed=1))
        return trials

    def test_no_cap_returns_everything(self):
        trials = self.make(10)
        self.assertEqual(len(replay_subset(trials, None)), 10)
        self.assertEqual(len(replay_subset(trials, 100)), 10)

    def test_cap_keeps_both_tails(self):
        trials = self.make(40)
        picked = replay_subset(trials, 8)
        self.assertEqual(len(picked), 8)
        ids = {t.trial_id for t in picked}
        # highest-scoring impostors and the lowest genuine must survive.
        self.assertIn(39, ids)
        self.assertIn(0, ids)
        self.assertEqual(sorted(t.trial_id for t in picked),
                         [t.trial_id for t in picked])

    def test_failures_are_never_dropped(self):
        trials = self.make(40)
        trials[17].expected_scores = [-1]
        picked = replay_subset(trials, 4)
        self.assertIn(17, {t.trial_id for t in picked})


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description='Cross-lock the sigfm-bench sweep against the real '
                    'virtual_image device.')
    p.add_argument('-i', '--input', metavar='DIR',
                   help='dataset directory of PGM impressions to sweep')
    p.add_argument('--smoke', metavar='DIR',
                   help='assert pipeline invariants on an existing export '
                        'directory; no sweep, no rates')
    p.add_argument('--selftest', action='store_true',
                   help='run the embedded unit cases and exit')
    p.add_argument('--worker', nargs=2, metavar=('JOB', 'OUT'),
                   help=argparse.SUPPRESS)
    p.add_argument('--builddir', default=os.path.join(SRC_ROOT, '_bench'),
                   metavar='DIR', help='meson build directory to test')
    p.add_argument('--bench', metavar='PATH',
                   help='sigfm-bench binary (default: inside --builddir)')
    p.add_argument('--dataset-id', metavar='NAME',
                   help='dataset label (default: basename of -i)')
    p.add_argument('--threshold', type=int, default=24, metavar='N',
                   help='operating threshold under test (default 24)')
    p.add_argument('--stages', type=int, default=5, metavar='N',
                   help='gallery size of the first round (default 5)')
    p.add_argument('--stages2', type=int, default=15, metavar='N',
                   help='gallery size of the second round (default 15)')
    p.add_argument('--probes', type=int, metavar='N',
                   help='probes per subject (default 3, or 2 under --quick)')
    p.add_argument('--subjects', type=int, metavar='N',
                   help='limit the number of subjects')
    p.add_argument('--seed', type=int, default=12345, metavar='N')
    p.add_argument('--replay-max', type=int, default=48, metavar='N',
                   help='cap on replayed trials per round (default 48)')
    p.add_argument('--export-dir', metavar='DIR',
                   help='keep the per-round exports (windows, manifest, '
                        'scores TSV) here instead of a temporary directory')
    p.add_argument('--quick', action='store_true',
                   help='reduced sweep, one gallery size')
    p.add_argument('--full-replay', action='store_true',
                   help='replay every exported trial, no cap')
    return p


def main(argv):
    opts = build_parser().parse_args(argv)

    if opts.worker:
        return worker_main(opts.worker[0], opts.worker[1])

    if opts.selftest:
        runner = unittest.TextTestRunner(stream=sys.stdout, verbosity=2)
        res = unittest.main(argv=[sys.argv[0]], exit=False, testRunner=runner)
        return 0 if res.result.wasSuccessful() else 1

    if bool(opts.smoke) == bool(opts.input):
        print('give exactly one of --smoke <export-dir>, -i <dataset-dir>, '
              'or --selftest', file=sys.stderr)
        return 2

    if opts.probes is None:
        opts.probes = 2 if opts.quick else 3
    if opts.quick and opts.subjects is None:
        opts.subjects = 20

    try:
        if opts.smoke:
            return mode_smoke(opts)
        return mode_full(opts)
    except Skip as e:
        print('Missing dependencies: %s' % e)
        return EXIT_SKIP
    except Failure as e:
        print('benchmark-matching: %s' % e, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
