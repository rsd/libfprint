#!/usr/bin/env python3
#
# Outside emulation mode the driver accepts only the real module: replays the
# first open of custom.pcapng with FP_DEVICE_EMULATION removed and checks that
# the synthetic data/module.bin is refused (DATA_INVALID, naming its sha256).
#
# Run without arguments, it starts itself under umockdev-run; --replayed is
# the part that runs inside the replay.

import hashlib
import os
import subprocess
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, 'data')


def run_replay():
    try:
        version = subprocess.check_output(['umockdev-run', '--version'])
    except FileNotFoundError:
        print('umockdev-run not found, skipping test!')
        return 77
    if tuple(int(_) for _ in version.split(b'.')[:3]) < (0, 16, 3):
        print('umockdev is too old for pcap replay, skipping test!')
        return 77

    device = os.path.join(HERE, 'device')
    with open(device) as f:
        syspath = '/sys' + f.readline().strip()[3:]

    # The point of this test: emulation mode off, whatever the caller set.
    env = dict(os.environ)
    env.pop('FP_DEVICE_EMULATION', None)

    wrapper = os.getenv('LIBFPRINT_TEST_WRAPPER')
    return subprocess.call(['umockdev-run', '-d', device,
                            '-p', '%s=%s' % (syspath, os.path.join(HERE, 'custom.pcapng')),
                            '--'] +
                           (wrapper.split(' ') if wrapper else []) +
                           [sys.executable, os.path.abspath(__file__), '--replayed'],
                           env=env)


def replayed():
    assert os.getenv('FP_DEVICE_EMULATION') is None

    os.environ['FP_SYNAPTICS0078_DATA_DIR'] = DATA_DIR
    os.environ['OPENCV_OPENCL_RUNTIME'] = 'disabled'

    import gi
    gi.require_version('FPrint', '2.0')
    from gi.repository import FPrint, GLib

    sys.excepthook = lambda *args: (traceback.print_exception(*args), sys.exit(1))

    with open(os.path.join(DATA_DIR, 'module.bin'), 'rb') as f:
        synthetic = hashlib.sha256(f.read()).hexdigest()

    c = FPrint.Context()
    c.enumerate()
    devices = c.get_devices()

    d = devices[0]
    del devices
    assert d.get_driver() == "synaptics0078"

    try:
        d.open_sync()
        d.close_sync()
        raise AssertionError("the synthetic module was accepted outside emulation mode")
    except GLib.Error as e:
        print("open failed:", e.message)
        assert e.matches(FPrint.device_error_quark(), FPrint.DeviceError.DATA_INVALID)
        assert ("module.bin has sha256 %s, expected " % synthetic) in e.message
        pinned = e.message.rsplit("expected ", 1)[1]
        assert len(pinned) == 64 and pinned != synthetic, pinned

    del d
    del c
    return 0


if __name__ == '__main__':
    sys.exit(replayed() if sys.argv[1:] == ['--replayed'] else run_replay())
