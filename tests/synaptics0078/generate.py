#!/usr/bin/env python3
#
# Copyright (C) 2026 Raul Dias <raul@dias.com.br>
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

"""Writes the synaptics0078 umockdev replay test data; nothing is recorded.

Every byte is a protocol constant the driver spells, a seeded synthetic
stand-in, or a public USB descriptor field:

  device                          umockdev device description (06cb:0078)
  custom.pcapng                   the USB session custom.py replays
  data/module.bin                 synthetic capture module (not firmware)
  data/program-e5171a0cffee.bin   synthetic capture program

The module, the program and the images are splitmix64 output: an ASCII tag
plus filler, and seeded Gaussian blob fields (no ridges, no minutiae). A wait
that has to time out or be cancelled is a submit with no completion, so
libusb's own timer or cancel ends it under replay.

  generate.py [--output DIR]          write the files (default: next to this
                                      script)
  generate.py --check [--output DIR]  regenerate into a temporary directory,
                                      compare byte for byte, and check that
                                      the driver's SYNAPTICS0078_TEST_MODULE_SHA256
                                      is the sha256 of data/module.bin

Python standard library only; the output does not depend on the machine,
the time or the environment.
"""

import argparse
import filecmp
import hashlib
import math
import os
import re
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER_SOURCE = os.path.join(HERE, '..', '..', 'libfprint', 'drivers',
                             'synaptics0078.c')

MASK64 = (1 << 64) - 1

# Images: native size, as the sensor returns them (the driver upscales 2x).
WIDTH = 144
HEIGHT = 56
UPSCALE = 2

MODULE_SIZE = 10500
PROGRAM_SIZE = 18869
MODULE_TAG = b'libfprint synaptics0078 synthetic test module, not firmware\n'
PROGRAM_TAG = (b'libfprint synaptics0078 synthetic test program, '
               b'not a capture program\n')

# RomInfo bytes 0x12..0x17; the driver swaps the nibbles of each byte.
SERIAL_BYTES = bytes.fromhex('5e71a1c0ffee')
SERIAL = ''.join('%02x' % (((b << 4) | (b >> 4)) & 0xff) for b in SERIAL_BYTES)
PROGRAM_FILE = 'program-%s.bin' % SERIAL

# Which synthetic image custom.py expects where.
CAPTURE_IMAGE = 0
VERIFY_MATCH_IMAGE = 0
VERIFY_OTHER_IMAGE = 40
ENROLL_STAGES = 18

# The driver's same-touch threshold (mean absolute difference, native pixels).
NEW_TOUCH_DIFF = 30

# Timings of the driver, only used to space the timestamps of waits that end
# without a completion (umockdev replays no delays).
PROBE_WINDOW_MS = 500
PROBE_PAUSE_MS = 250
IMAGE_REREAD_MS = 1000

# USB topology of the device file; the pcapng carries the same numbers.
BUSNUM = 1
DEVNUM = 2
SYSPATH = '/devices/pci0000:00/0000:00:14.0/usb1/1-3'

EP_OUT = 0x01
EP_IN = 0x81
EP_INT = 0x83
READ_SIZE = 1024
PROGRAM_READ_SIZE = 0x1000
IMAGE_READ_SIZE = 16384
INT_READ_SIZE = 8
INT_INTERVAL = 4

URB_INTERRUPT = 1
URB_BULK = 3
EINPROGRESS = 115

LINKTYPE_USB_LINUX_MMAPPED = 220
# 2025-10-09 08:53:20 UTC: fixed, so the file does not depend on the clock.
START_US = 1760000000 * 1000000

# EP 0x83 events, as the driver's top-of-file comment describes them.
EVENT_NONE = bytes.fromhex('0000000000')       # after every 02, ignored
EVENT_FINGER = bytes.fromhex('0200401000')
EVENT_STARTED = bytes.fromhex('0340010000')    # kind 3 code 1, ignored
EVENT_READY = bytes.fromhex('0342040040')      # kind 3 code 4
EVENT_EXTEND = bytes.fromhex('0342050040')     # kind 3 code 5

CMD_IMAGE = bytes.fromhex('5100200000')
STATUS_OK = 0x0000
STATUS_NOT_READY = 0x0001
STATUS_CAPTURE_ERROR = 0x0007
STATUS_NO_CAPTURE = 0x0412


def splitmix64(state):
    while True:
        state = (state + 0x9E3779B97F4A7C15) & MASK64
        z = state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
        yield z ^ (z >> 31)


def filler(seed, size):
    gen = splitmix64(seed)
    out = bytearray()
    while len(out) < size:
        out += struct.pack('<Q', next(gen))
    return bytes(out[:size])


def module():
    """The synthetic module.bin: a tag, then filler (no firmware)."""
    return MODULE_TAG + filler(0x0078_0D01E, MODULE_SIZE - len(MODULE_TAG))


def program():
    """The synthetic program file: 0x02, a tag, then filler."""
    head = b'\x02' + PROGRAM_TAG
    return head + filler(0x0078_0002, PROGRAM_SIZE - len(head))


def uniform(gen, lo, hi):
    return lo + (hi - lo) * (next(gen) >> 11) / float(1 << 53)


def image(k, blobs=90, smin=1.6, smax=4.0):
    """Synthetic image @k (144x56, 8 bit): seeded light and dark Gaussian
    blobs, scaled to mean 128 and standard deviation 64."""
    gen = splitmix64(0x5A0078 * 1000003 + k)
    field = [0.0] * (WIDTH * HEIGHT)
    for _ in range(blobs):
        cx, cy = uniform(gen, 0, WIDTH), uniform(gen, 0, HEIGHT)
        s = uniform(gen, smin, smax)
        a = uniform(gen, 0.5, 1.0) * (1 if next(gen) & 1 else -1)
        r = int(3 * s) + 1
        for y in range(max(0, int(cy) - r), min(HEIGHT, int(cy) + r + 1)):
            for x in range(max(0, int(cx) - r), min(WIDTH, int(cx) + r + 1)):
                field[y * WIDTH + x] += a * math.exp(
                    -((x - cx) ** 2 + (y - cy) ** 2) / (2 * s * s))
    mean = sum(field) / len(field)
    sd = math.sqrt(sum((v - mean) ** 2 for v in field) / len(field)) or 1.0
    return bytes(max(0, min(255, int(round(128 + 64 * (v - mean) / sd))))
                 for v in field)


def upscale(pixels):
    """Nearest-neighbour 2x upscale, as the driver hands images to the core."""
    width = WIDTH * UPSCALE
    out = bytearray()
    for y in range(HEIGHT * UPSCALE):
        row = pixels[(y // UPSCALE) * WIDTH:(y // UPSCALE + 1) * WIDTH]
        out += bytes(row[x // UPSCALE] for x in range(width))
    return bytes(out)


def image_diff(a, b):
    return sum(abs(x - y) for x, y in zip(a, b)) / len(a)


# ---------------------------------------------------------------- replies


def status(code, length=2):
    return struct.pack('<H', code) + bytes(length - 2)


def rom_info(module_loaded):
    """RomInfo (01 reply, 38 bytes): ROM 6.20, the synthetic serial and
    byte 17, the module state (0 = no module loaded)."""
    reply = bytearray(38)
    reply[10] = 6
    reply[11] = 0x14
    reply[13] = 0x41
    reply[17] = 0x0e if module_loaded else 0
    reply[0x12:0x18] = SERIAL_BYTES
    return bytes(reply)


def sensor_id():
    """75 reply: status, a reserved u32, then minor 0x0037, major 0x00b7."""
    return status(STATUS_OK) + bytes(4) + struct.pack('<HH', 0x0037, 0x00b7)


def image_reply(pixels, code=STATUS_OK, error=0):
    """51 reply: u16 status, u32 length, x, y, w1, w2, u32 error, pixels."""
    header = struct.pack('<HIHHHHI', code, 12 + len(pixels), WIDTH, HEIGHT,
                         0, 0, error)
    return header + pixels


# ---------------------------------------------------------------- pcapng


class Replay:
    """usbmon events (LINKTYPE_USB_LINUX_MMAPPED) of one USB session."""

    def __init__(self):
        self.packets = []
        self.urb_id = 0xffff888100000000
        self.time_us = START_US

    def _event(self, urb_id, kind, xfer, endpoint, code, length, data=b''):
        if data:
            data_flag = 0
        else:
            data_flag = ord('<') if endpoint & 0x80 else ord('>')
        header = struct.pack('<QBBBBHbbqiiII8siiII',
                             urb_id, ord(kind), xfer, endpoint, DEVNUM, BUSNUM,
                             ord('-'), data_flag,
                             self.time_us // 1000000, self.time_us % 1000000,
                             code, length, len(data), bytes(8),
                             INT_INTERVAL if xfer == URB_INTERRUPT else 0, 0,
                             0x200 if endpoint & 0x80 else 0, 0)
        assert len(header) == 64
        self.packets.append((self.time_us, header + data))
        self.time_us += 1000

    def _submit(self, xfer, endpoint, length, data=b''):
        self.urb_id += 0x100
        self._event(self.urb_id, 'S', xfer, endpoint, -EINPROGRESS, length,
                    data)
        return self.urb_id

    def bulk_out(self, data):
        urb = self._submit(URB_BULK, EP_OUT, len(data), data)
        self._event(urb, 'C', URB_BULK, EP_OUT, 0, len(data))

    def bulk_in(self, size, reply):
        urb = self._submit(URB_BULK, EP_IN, size)
        self._event(urb, 'C', URB_BULK, EP_IN, 0, len(reply), reply)

    def interrupt(self, event):
        urb = self._submit(URB_INTERRUPT, EP_INT, INT_READ_SIZE)
        self._event(urb, 'C', URB_INTERRUPT, EP_INT, 0, len(event), event)

    def no_reply(self, xfer, endpoint, size, wait_ms):
        """A submit that never completes: the driver's timeout or a cancel
        ends it, so libusb reports TIMED_OUT or CANCELLED itself."""
        self._submit(xfer, endpoint, size)
        self.time_us += wait_ms * 1000

    def command(self, data, reply, read_size=READ_SIZE):
        self.bulk_out(data)
        self.bulk_in(read_size, reply)

    def pcapng(self):
        def block(block_type, body):
            body += bytes(-len(body) % 4)
            length = 12 + len(body)
            return (struct.pack('<II', block_type, length) + body +
                    struct.pack('<I', length))

        out = block(0x0A0D0D0A, struct.pack('<IHHq', 0x1A2B3C4D, 1, 0, -1))
        out += block(0x00000001, struct.pack('<HHI', LINKTYPE_USB_LINUX_MMAPPED,
                                             0, 0x40000))
        for time_us, data in self.packets:
            out += block(0x00000006,
                         struct.pack('<IIIII', 0, time_us >> 32,
                                     time_us & 0xffffffff, len(data),
                                     len(data)) + data)
        return out


# ---------------------------------------------------------------- session


class Session:
    """The driver's exchanges, step by step, as its SSMs run them."""

    def __init__(self):
        self.replay = Replay()
        self.reference = None    # native pixels of the last delivered image
        self.stale = False       # a retry since the reference was delivered

    def open(self, upload, load_ok=True):
        r = self.replay
        r.command(b'\x01', rom_info(module_loaded=not upload))
        r.command(b'\x19', status(STATUS_OK, 68))
        r.command(b'\x75', sensor_id())
        if load_ok and upload:
            self.upload()

    def upload(self):
        r = self.replay
        r.command(b'\x06' + module(), status(STATUS_OK))
        r.command(b'\x01', rom_info(module_loaded=True))

    def activate(self, upload=False):
        self.replay.command(b'\x01', rom_info(module_loaded=not upload))
        if upload:
            self.upload()

    def arm(self, event=True):
        """02 + program; the sensor reports 0000000000 right after it."""
        self.replay.command(program(), status(STATUS_OK, 2154),
                            PROGRAM_READ_SIZE)
        if event:
            self.replay.interrupt(EVENT_NONE)

    def disarm(self, capture_ended):
        """04 answers 0x0412 once the capture has ended, 0 while it waits."""
        self.replay.command(b'\x04', status(STATUS_NO_CAPTURE if capture_ended
                                            else STATUS_OK))
        self.replay.command(b'\x52', status(STATUS_OK))

    def touch(self, events=(EVENT_FINGER, EVENT_STARTED, EVENT_READY)):
        for event in events:
            self.replay.interrupt(event)

    def read_image(self, reply):
        self.replay.command(CMD_IMAGE, reply, IMAGE_READ_SIZE)

    def deliver(self, pixels, guard=True):
        """A touch whose image the driver hands to the core."""
        if guard and self.reference is not None and not self.stale:
            assert image_diff(pixels, self.reference) >= NEW_TOUCH_DIFF
        self.read_image(image_reply(pixels))
        self.disarm(capture_ended=True)
        self.reference = pixels
        self.stale = False

    def probe_no_finger(self):
        """A finger-off probe that sees no finger: the window times out and
        the sensor stays armed."""
        self.arm()
        self.replay.no_reply(URB_INTERRUPT, EP_INT, INT_READ_SIZE,
                             PROBE_WINDOW_MS)

    def listen_capture(self, k):
        """A capture on the armed sensor, then finger off."""
        self.touch()
        self.deliver(image(k))
        self.probe_no_finger()


def scenario():
    """The session custom.py runs, step by step; see the README."""
    s = Session()
    r = s.replay

    # 1. open with a corrupted module.bin: DATA_INVALID after 01, 19, 75
    s.open(upload=True, load_ok=False)

    # 2. open: no module loaded (RomInfo byte 17 = 0), 06 + module, 01
    s.open(upload=True)

    # 3. capture_sync: image CAPTURE_IMAGE, finger off, deactivation
    s.activate()
    s.arm()
    s.touch()
    s.deliver(image(CAPTURE_IMAGE))
    s.probe_no_finger()
    s.disarm(capture_ended=False)

    # 4. enroll, 18 stages
    s.activate()

    # stage 1: the same touch as the capture before: the guard keeps it,
    # waits for the lift (a probe without a finger), then takes the next
    # touch unchecked (it is the same image again, so the bypass is visible)
    s.arm()
    s.touch()
    assert image_diff(image(0), s.reference) < NEW_TOUCH_DIFF
    s.read_image(image_reply(image(0)))
    s.disarm(capture_ended=True)
    r.time_us += PROBE_PAUSE_MS * 1000
    s.probe_no_finger()
    s.touch()
    s.deliver(image(0), guard=False)
    s.probe_no_finger()

    # stage 2
    s.listen_capture(1)

    # stage 3, attempt 1: the 51 reply carries an error header: a retry
    s.touch()
    s.read_image(image_reply(bytes(WIDTH * HEIGHT), STATUS_CAPTURE_ERROR,
                             STATUS_CAPTURE_ERROR))
    s.disarm(capture_ended=True)
    s.stale = True
    s.probe_no_finger()

    # stage 3: the reference is stale, so the guard is skipped: the very
    # touch of stage 2 is taken
    s.touch()
    s.deliver(image(1))
    s.probe_no_finger()

    # stage 4: "not ready" on 51, the re-read times out, 51 again
    s.touch()
    s.read_image(status(STATUS_NOT_READY))
    r.no_reply(URB_BULK, EP_IN, IMAGE_READ_SIZE, IMAGE_REREAD_MS)
    s.deliver(image(2))
    s.probe_no_finger()

    # stage 5: an "extend" event (code 5) before "ready"
    s.touch((EVENT_FINGER, EVENT_STARTED, EVENT_EXTEND, EVENT_READY))
    s.deliver(image(3))
    s.probe_no_finger()

    # stages 6..17
    for k in range(4, 16):
        s.listen_capture(k)

    # stage 18: the core completes the enroll and deactivates as soon as
    # minutiae detection ends, which may be before, during or after the
    # finger-off probe's EP 0x83 wait. That wait has no submit here, so the
    # replay goes on to 04/52 in every case.
    s.touch()
    s.deliver(image(16))
    s.arm(event=False)
    s.disarm(capture_ended=False)

    # 5. verify, matching: no module loaded at activation, uploaded again
    s.activate(upload=True)
    s.arm()
    s.touch()
    s.deliver(image(VERIFY_MATCH_IMAGE))
    s.probe_no_finger()
    s.disarm(capture_ended=False)

    # 6. verify, not matching
    s.activate()
    s.arm()
    s.touch()
    s.deliver(image(VERIFY_OTHER_IMAGE))
    s.probe_no_finger()
    s.disarm(capture_ended=False)

    # 7. close: no USB traffic
    return r


# ---------------------------------------------------------------- device


def descriptors():
    """Device and configuration descriptors (public fields of 06cb:0078)."""
    endpoints = [(0x01, 2, 64, 0), (0x81, 2, 64, 0), (0x82, 2, 64, 0),
                 (0x83, 3, 8, 4), (0x84, 3, 16, 10)]
    device = struct.pack('<BBHBBBBHHHBBBB', 18, 1, 0x0200, 0xff, 0x10, 0xff, 8,
                         0x06cb, 0x0078, 0x0154, 0, 0, 1, 1)
    interface = struct.pack('<BBBBBBBBB', 9, 4, 0, 0, len(endpoints), 0xff, 0,
                            0, 0)
    eps = b''.join(struct.pack('<BBBBHB', 7, 5, ep, attr, size, interval)
                   for ep, attr, size, interval in endpoints)
    total = 9 + len(interface) + len(eps)
    config = struct.pack('<BBHBBBBB', 9, 2, total, 1, 1, 0, 0xa0, 50)
    return (device + config + interface + eps).hex().upper()


def root_hub_descriptors():
    """A generic Linux USB 2.0 root hub (1d6b:0002)."""
    device = struct.pack('<BBHBBBBHHHBBBB', 18, 1, 0x0200, 9, 0, 1, 64,
                         0x1d6b, 0x0002, 0x0600, 3, 2, 1, 1)
    config = struct.pack('<BBHBBBBB', 9, 2, 25, 1, 1, 0, 0xe0, 0)
    interface = struct.pack('<BBBBBBBBB', 9, 4, 0, 0, 1, 9, 0, 0, 0)
    endpoint = struct.pack('<BBBBHB', 7, 5, 0x81, 3, 4, 12)
    return (device + config + interface + endpoint).hex().upper()


def device_file():
    port = SYSPATH.rsplit('-', 1)[1]
    hub = SYSPATH.rsplit('/', 1)[0]
    lines = [
        'P: ' + SYSPATH,
        'N: bus/usb/%03d/%03d=%s' % (BUSNUM, DEVNUM, descriptors()),
        'E: DEVNAME=/dev/bus/usb/%03d/%03d' % (BUSNUM, DEVNUM),
        'E: DEVTYPE=usb_device',
        'E: DRIVER=usb',
        'E: PRODUCT=6cb/78/154',
        'E: TYPE=255/16/255',
        'E: BUSNUM=%03d' % BUSNUM,
        'E: DEVNUM=%03d' % DEVNUM,
        'E: MAJOR=189',
        'E: MINOR=%d' % ((BUSNUM - 1) * 128 + DEVNUM - 1),
        'E: SUBSYSTEM=usb',
        'E: ID_VENDOR=06cb',
        'E: ID_VENDOR_ENC=06cb',
        'E: ID_VENDOR_ID=06cb',
        'E: ID_MODEL=0078',
        'E: ID_MODEL_ENC=0078',
        'E: ID_MODEL_ID=0078',
        'E: ID_REVISION=0154',
        'E: ID_SERIAL=06cb_0078_' + SERIAL,
        'E: ID_SERIAL_SHORT=' + SERIAL,
        'E: ID_BUS=usb',
        'E: ID_USB_INTERFACES=:ff0000:',
        'E: ID_VENDOR_FROM_DATABASE=Synaptics, Inc.',
        'E: ID_PATH=pci-0000:00:14.0-usb-0:' + port,
        'E: ID_PATH_TAG=pci-0000_00_14_0-usb-0_' + port,
        'A: authorized=1\\n',
        'A: avoid_reset_quirk=0\\n',
        'A: bConfigurationValue=1\\n',
        'A: bDeviceClass=ff\\n',
        'A: bDeviceProtocol=ff\\n',
        'A: bDeviceSubClass=10\\n',
        'A: bMaxPacketSize0=8\\n',
        'A: bMaxPower=100mA\\n',
        'A: bNumConfigurations=1\\n',
        'A: bNumInterfaces= 1\\n',
        'A: bcdDevice=0154\\n',
        'A: bmAttributes=a0\\n',
        'A: busnum=%d\\n' % BUSNUM,
        'A: configuration=',
        'H: descriptors=' + descriptors(),
        'A: dev=189:%d\\n' % ((BUSNUM - 1) * 128 + DEVNUM - 1),
        'A: devnum=%d\\n' % DEVNUM,
        'A: devpath=%s\\n' % port,
        'L: driver=../../../../../bus/usb/drivers/usb',
        'A: idProduct=0078\\n',
        'A: idVendor=06cb\\n',
        'A: ltm_capable=no\\n',
        'A: maxchild=0\\n',
        'L: port=../%d-0:1.0/usb%d-port%s' % (BUSNUM, BUSNUM, port),
        'A: power/autosuspend_delay_ms=2000\\n',
        'A: power/control=auto\\n',
        'A: power/persist=1\\n',
        'A: power/runtime_status=active\\n',
        'A: power/wakeup=disabled\\n',
        'A: quirks=0x0\\n',
        'A: removable=fixed\\n',
        'A: rx_lanes=1\\n',
        'A: serial=%s\\n' % SERIAL,
        'A: speed=12\\n',
        'A: tx_lanes=1\\n',
        'A: version= 2.00\\n',
        '',
        'P: ' + hub,
        'N: bus/usb/%03d/001=%s' % (BUSNUM, root_hub_descriptors()),
        'E: DEVNAME=/dev/bus/usb/%03d/001' % BUSNUM,
        'E: DEVTYPE=usb_device',
        'E: DRIVER=usb',
        'E: PRODUCT=1d6b/2/600',
        'E: TYPE=9/0/1',
        'E: BUSNUM=%03d' % BUSNUM,
        'E: DEVNUM=001',
        'E: MAJOR=189',
        'E: MINOR=%d' % ((BUSNUM - 1) * 128),
        'E: SUBSYSTEM=usb',
        'E: ID_VENDOR_ID=1d6b',
        'E: ID_MODEL_ID=0002',
        'E: ID_BUS=usb',
        'E: ID_USB_INTERFACES=:090000:',
        'E: ID_PATH=pci-0000:00:14.0',
        'E: ID_PATH_TAG=pci-0000_00_14_0',
        'A: authorized=1\\n',
        'A: bConfigurationValue=1\\n',
        'A: bDeviceClass=09\\n',
        'A: bDeviceProtocol=01\\n',
        'A: bDeviceSubClass=00\\n',
        'A: bMaxPacketSize0=64\\n',
        'A: bNumConfigurations=1\\n',
        'A: bNumInterfaces= 1\\n',
        'A: bcdDevice=0600\\n',
        'A: busnum=%d\\n' % BUSNUM,
        'H: descriptors=' + root_hub_descriptors(),
        'A: dev=189:%d\\n' % ((BUSNUM - 1) * 128),
        'A: devnum=1\\n',
        'A: devpath=0\\n',
        'L: driver=../../../../bus/usb/drivers/usb',
        'A: idProduct=0002\\n',
        'A: idVendor=1d6b\\n',
        'A: maxchild=16\\n',
        'A: product=xHCI Host Controller\\n',
        'A: serial=0000:00:14.0\\n',
        'A: speed=480\\n',
        'A: version= 2.00\\n',
        '',
        'P: ' + hub.rsplit('/', 1)[0],
        'E: DRIVER=xhci_hcd',
        'E: PCI_CLASS=C0330',
        'E: SUBSYSTEM=pci',
        'A: class=0x0c0330\\n',
        'L: driver=../../../bus/pci/drivers/xhci_hcd',
        '',
    ]
    return '\n'.join(lines)


# ---------------------------------------------------------------- output


def files():
    """Relative path -> contents of every generated file."""
    return {
        'device': device_file().encode('ascii'),
        'custom.pcapng': scenario().pcapng(),
        os.path.join('data', 'module.bin'): module(),
        os.path.join('data', PROGRAM_FILE): program(),
    }


def write(outdir, generated=None):
    if generated is None:
        generated = files()
    for name, data in sorted(generated.items()):
        path = os.path.join(outdir, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(data)


def driver_test_sha256(path):
    with open(path, encoding='utf-8') as f:
        source = f.read()
    m = re.search(r'#define\s+SYNAPTICS0078_TEST_MODULE_SHA256\s*\\?\s*'
                  r'"([0-9a-f]{64})"', source)
    if not m:
        raise SystemExit('%s: no SYNAPTICS0078_TEST_MODULE_SHA256 define'
                         % path)
    return m.group(1)


def check(fixture_dir, driver):
    failures = []
    with tempfile.TemporaryDirectory(prefix='synaptics0078-check-') as tmp:
        generated = files()
        write(tmp, generated)
        for name in sorted(generated):
            committed = os.path.join(fixture_dir, name)
            if not os.path.isfile(committed):
                failures.append('%s is missing' % name)
            elif not filecmp.cmp(os.path.join(tmp, name), committed,
                                 shallow=False):
                failures.append('%s differs from the generated file' % name)

    module_sha = hashlib.sha256(module()).hexdigest()
    pinned = driver_test_sha256(driver)
    if pinned != module_sha:
        failures.append('SYNAPTICS0078_TEST_MODULE_SHA256 in %s is %s, '
                        'data/module.bin has sha256 %s'
                        % (driver, pinned, module_sha))

    for failure in failures:
        print('FAIL:', failure)
    if failures:
        print('Run %s to regenerate the test data.' % os.path.basename(__file__))
        return 1

    print('OK: test data is generated by this script; the driver pins '
          'module sha256 %s' % module_sha)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--output', default=HERE,
                        help='fixture directory (default: next to this script)')
    parser.add_argument('--check', action='store_true',
                        help='compare instead of writing')
    parser.add_argument('--driver', default=DRIVER_SOURCE,
                        help='driver source holding the test module pin '
                             '(--check only)')
    args = parser.parse_args()

    if args.check:
        return check(args.output, args.driver)

    write(args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
