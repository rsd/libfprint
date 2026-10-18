#!/usr/bin/env python3
#
# Replays the session generate.py synthesizes; see README.md.

import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, 'data')

# Before FPrint is loaded: the driver reads its data files from here, and
# OpenCV must not pick an OpenCL device.
os.environ['FP_SYNAPTICS0078_DATA_DIR'] = DATA_DIR
os.environ['OPENCV_OPENCL_RUNTIME'] = 'disabled'

# Each finger-off probe waits 500 ms from the 02 reply. Under a wrapper
# (valgrind, nice, ...) the main loop can stall longer than that and the driver
# would report finger-off without the submit the replay expects, hanging the
# test. So with a wrapper the window is widened, unless the user chose one.
if os.environ.get('LIBFPRINT_TEST_WRAPPER'):
    os.environ.setdefault('FP_SYNAPTICS0078_PROBE_WINDOW_MS', '5000')

sys.dont_write_bytecode = True
import generate

import gi
gi.require_version('FPrint', '2.0')
from gi.repository import FPrint, GLib

# Exit with error on any exception, included those happening in async callbacks
sys.excepthook = lambda *args: (traceback.print_exception(*args), sys.exit(1))

c = FPrint.Context()
c.enumerate()
devices = c.get_devices()

d = devices[0]
del devices

assert d.get_driver() == "synaptics0078"
assert d.has_feature(FPrint.DeviceFeature.CAPTURE)
assert d.has_feature(FPrint.DeviceFeature.IDENTIFY)
assert d.has_feature(FPrint.DeviceFeature.VERIFY)
assert not d.has_feature(FPrint.DeviceFeature.DUPLICATES_CHECK)
assert not d.has_feature(FPrint.DeviceFeature.STORAGE)
assert not d.has_feature(FPrint.DeviceFeature.STORAGE_LIST)
assert not d.has_feature(FPrint.DeviceFeature.STORAGE_DELETE)
assert not d.has_feature(FPrint.DeviceFeature.STORAGE_CLEAR)
assert d.get_nr_enroll_stages() == generate.ENROLL_STAGES

# 1. A module.bin with one byte flipped is refused, test pin or not, and the
# failed open leaves the device ready for the next one.
print("--- OPEN WITH A CORRUPTED MODULE ---")
with tempfile.TemporaryDirectory(prefix='synaptics0078-') as bad_dir:
    module = bytearray(generate.module())
    module[len(module) // 2] ^= 0x01
    with open(os.path.join(bad_dir, 'module.bin'), 'wb') as f:
        f.write(module)
    with open(os.path.join(bad_dir, generate.PROGRAM_FILE), 'wb') as f:
        f.write(generate.program())

    os.environ['FP_SYNAPTICS0078_DATA_DIR'] = bad_dir
    try:
        d.open_sync()
        raise AssertionError("open with a corrupted module succeeded")
    except GLib.Error as e:
        print("open failed:", e.message)
        assert e.matches(FPrint.device_error_quark(), FPrint.DeviceError.DATA_INVALID)
        assert "has sha256" in e.message
    finally:
        os.environ['FP_SYNAPTICS0078_DATA_DIR'] = DATA_DIR

# 2. Open: no module loaded yet, so it is uploaded.
print("--- OPEN ---")
d.open_sync()

try:
    # 3. Capture: the image is the 2x upscale of the synthetic image.
    print("--- CAPTURE ---")
    assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
    img = d.capture_sync(True)
    assert img.get_width() == generate.WIDTH * generate.UPSCALE
    assert img.get_height() == generate.HEIGHT * generate.UPSCALE
    assert bytes(img.get_data()) == generate.upscale(generate.image(generate.CAPTURE_IMAGE))
    del img
    print("--- CAPTURE DONE ---")

    # 4. Enroll: 18 stages, one of them retried.
    print("--- ENROLLING ---")
    stages = []
    retries = []

    def enroll_progress(dev, stage, pnt, data, error):
        print("enroll progress: stage %d%s" % (stage, ", retry: " + error.message if error else ""))
        if error:
            assert error.matches(FPrint.device_retry_quark(), FPrint.DeviceRetry.GENERAL)
            retries.append(stage)
        else:
            stages.append(stage)

    template = FPrint.Print.new(d)
    p = d.enroll_sync(template, None, enroll_progress, None)
    assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
    assert stages == list(range(1, generate.ENROLL_STAGES + 1)), stages
    assert retries == [2], retries
    del template
    print("--- ENROLL DONE ---")

    # 5. Verify the enrolled touch: a match (the module is uploaded again at
    # activation in the replay).
    print("--- VERIFYING (MATCH) ---")
    verify_res, verify_print = d.verify_sync(p)
    assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
    print("--- VERIFY DONE: Result %s ---" % verify_res)
    assert verify_res is True

    # 6. Verify an image that was never enrolled: no match.
    print("--- VERIFYING (NO MATCH) ---")
    verify_res, verify_print = d.verify_sync(p)
    assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
    print("--- VERIFY DONE: Result %s ---" % verify_res)
    assert verify_res is False
    del verify_print
    del p
finally:
    # 7. Close
    d.close_sync()

del d
del c
