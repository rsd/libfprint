synaptics0078 replay test
=========================

Everything in this directory is **synthetic**. `generate.py` writes it; no
part of it was recorded from a sensor, and it holds no firmware, no capture
program, no calibration and no fingerprint image.

| file | what it is |
|---|---|
| `generate.py` | the generator (Python standard library only) |
| `custom.py` | the test (`meson test synaptics0078`) |
| `hwpin.py` | the module pin outside emulation mode (`meson test synaptics0078-hwpin`) |
| `device` | the umockdev description of a 06cb:0078 (public descriptor fields, synthetic serial `e5171a0cffee`) |
| `custom.pcapng` | the USB session, in the usbmon format umockdev replays |
| `data/module.bin` | a stand-in for the capture module: an ASCII tag and seeded filler |
| `data/program-e5171a0cffee.bin` | a stand-in for the capture program: `0x02`, an ASCII tag and seeded filler |

The images are seeded fields of Gaussian blobs (144x56, no ridges, no
minutiae). They are distinct enough for the driver's same-touch guard and
SIGFM matches an image against itself with a margin of four orders of
magnitude, so the test checks outcomes (match, no match), never scores.

Regenerating and checking
-------------------------

```sh
tests/synaptics0078/generate.py           # rewrite the files
tests/synaptics0078/generate.py --check   # regenerate into a temporary directory and compare
```

The output is the same on every run. `--check` (the `synaptics0078-fixture`
test) also fails when `SYNAPTICS0078_TEST_MODULE_SHA256` in the driver is not
the sha256 of the generated `data/module.bin`. Changing the scenario, the
module or the images means: edit `generate.py`, run it, update the define if
the module changed, and commit the regenerated files with it.

The driver accepts the synthetic module only in emulation mode
(`FP_DEVICE_EMULATION=1` with the test emulation library loaded, as
`tests/meson.build` sets up). `hwpin.py` replays the first open of
`custom.pcapng` without `FP_DEVICE_EMULATION` and checks that the module is
refused with `DATA_INVALID`: outside the tests the real module pin stays the
only one accepted.

Timing limit
------------

Each finger-off probe waits 500 ms (`FP_SYNAPTICS0078_PROBE_WINDOW_MS`, the
driver's default) from the `02` reply, and the replay assumes the main loop
keeps up. A stall of more than 500 ms (valgrind, a loaded machine) makes the
driver report finger-off without the submit the replay expects, and the test
hangs until its 60 s timeout. So when `LIBFPRINT_TEST_WRAPPER` is set and not
empty, `custom.py` sets the variable to 5000 ms, unless it is already set;
without a wrapper the 500 ms default is untouched. A probe with no event
lasts its whole window, so the test then takes about two minutes (23 probes):
run it with a timeout multiplier, as the `valgrind` test setup does (20).
`hwpin.py` runs no probe
(the open is refused at the module check) and needs nothing.

What the session covers
-----------------------

1. open with a corrupted module: `DATA_INVALID`; then open again with the
   module upload (`06`) and its confirmation;
2. a capture, compared with the 2x upscale of synthetic image 0;
3. an 18-stage enroll with one each of: the same-touch guard (a probe without
   a finger, then the lift bypass), a `51` reply with an error header (a retry,
   after which the reference is stale and the guard is skipped), "not ready"
   on `51` with a re-read that times out, an "extend" event, and `0x0412`
   ("no capture in progress") on `04`, which every delivered image produces;
4. a matching verify, with the module uploaded again at activation;
5. a verify of an image that was never enrolled: no match;
6. close.

Why it is not recorded
----------------------

- umockdev returns every recorded completion at once, so a timed-out read
  in a recording replays as an immediate empty read. This driver relies on
  timeouts: a finger-off probe is a window with no event. Here a wait that
  must time out or be cancelled is a submit with no completion, so libusb's
  own timer or cancel ends it, as on hardware.
- A recording carries the real module, the per-unit program and fingerprint
  images, none of which can be published.

One timing is not under the replay's control: at the last enroll stage the
core deactivates as soon as minutiae detection is done, which can be before
or after the finger-off probe starts its `EP 0x83` wait. That wait has no
submit in the file, so both orders replay the same `04`/`52`; when the wait
was started, umockdev logs "Replay may be stuck: Reaping discard URB" as the
deactivation cancels it, which is expected.
