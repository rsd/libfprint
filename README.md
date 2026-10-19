# libfprint with Synaptics 06cb:0078 support

This is a fork of [libfprint](https://gitlab.freedesktop.org/libfprint/libfprint) that adds a driver for the Synaptics fingerprint reader with USB ID `06cb:0078`. It was developed and tested on a Clevo P870KM laptop (model string P870KM_KM1). Other laptops with the same USB ID should work too, but they are untested.

## Why a fork

libfprint, the library Linux desktops use for fingerprint readers, has no driver for this reader, so it did not work on Linux. Making it work needed three things that upstream libfprint does not have yet.

- **A matcher that works on this sensor.** The reader returns a very small 144x56 pixel image. libfprint's built-in matcher (NBIS) rejected every genuine attempt in our tests, while keypoint matchers told the fingers apart. That matcher is SIGFM, proposed upstream in [MR !530](https://gitlab.freedesktop.org/libfprint/libfprint/-/merge_requests/530) and not merged. This fork carries SIGFM so the driver can match.
- **Vendor and per-unit data at runtime.** The reader needs a vendor firmware module and a capture program made for each individual unit. It keeps the module only in RAM and loses it on a reboot, so the driver re-sends it whenever the reader reports it missing, and it sends the program with every capture. Neither is shipped inside libfprint: the module is proprietary and the program is unique to each unit. libfprint has no place yet for that kind of per-device data (see [issue #868](https://gitlab.freedesktop.org/libfprint/libfprint/-/issues/868) and [MR !368](https://gitlab.freedesktop.org/libfprint/libfprint/-/merge_requests/368)). The [companion tool](https://github.com/rsd/fingerprint-06cb-0078) fills that gap. Its `sudo synaptics0078-setup install` downloads the vendor's public driver package (from Microsoft's Update Catalog, or a fallback copy), takes the module out of it, and calibrates your reader to build its program, all on your machine. No Windows is needed.
- **One small core fix.** An enrollment that finishes while the device still waits for the finger to lift raised a spurious warning. The driver needs that fixed in `libfprint/fpi-image-device.c`.

## What differs from upstream

The fork sits on upstream commit [6f9479c3](https://gitlab.freedesktop.org/libfprint/libfprint/-/commit/6f9479c3d55f847c1b3769f28ceb99227f9858cf). On top of it:

- SIGFM, its benchmark tooling and tests: 8 commits by Sergey Subbotin, a revision of the SIGFM work proposed in MR !530. OpenCV stays an optional dependency.
- One commit by Omkar Chandorkar, "device: Validate gallery equality only for raw prints".
- The `synaptics0078` driver (about 2600 lines), by Raul Dias.
- The `fpi-image-device.c` fix above, by Raul Dias.
- A replay test for the driver using umockdev, in `tests/synaptics0078/`, by Raul Dias.

Run `git log 6f9479c3..HEAD` for the exact list.

## A clean, compatible fork

- It sits on current upstream master as of October 2026.
- It removes no upstream driver, feature or public function. Changes to existing upstream code are small. The SIGFM commits rename the internal `bz3_threshold` field and its setter to `score_threshold` (one line in each image driver) and let the image-matching code pick a matcher per driver. Omkar Chandorkar's commit limits a gallery check to raw prints. The core fix silences one warning. The rest are additions to the build, CI and hardware-list files.
- The library name, soname (`libfprint-2.so.2`) and version (1.94.100) are those of upstream. No upstream public header or exported symbol is changed or removed; the fork only adds two exported SIGFM image helpers and the header `sigfm/sigfm.h`. So fprintd and other programs work unchanged.
- Other readers keep libfprint's standard matcher, NBIS. Only the `synaptics0078` driver selects SIGFM (the virtual test driver can also be switched to it for benchmarks).
- The AUR package builds every upstream driver (`-D drivers=all`) and runs the upstream test suite in its check step (egis_etu905, a known upstream flake, is rerun up to three times when it is the only failure).
- The package provides and conflicts with `libfprint` (and `libfprint-git`), so it replaces either cleanly. To go back, reinstall Arch's `libfprint` (`sudo pacman -S libfprint`). Prints enrolled with this reader will not work with stock libfprint.

## Install

On Arch Linux, install the AUR package [libfprint-sigfm-0078](https://aur.archlinux.org/packages/libfprint-sigfm-0078) and `fprintd` (`yay -S libfprint-sigfm-0078 fprintd`). It replaces Arch's `libfprint` (pacman asks to remove it). Then run the setup with your fingers off the reader, and enroll:

```sh
sudo synaptics0078-setup install
fprintd-enroll
```

On Omarchy, follow the Install section of the [companion repository](https://github.com/rsd/fingerprint-06cb-0078) instead: Omarchy's own fingerprint setup can otherwise replace this package.

There is no package for other distributions yet. The false-accept limits and all other details are in the [companion repository](https://github.com/rsd/fingerprint-06cb-0078).

## Relationship to upstream

The fork exists because SIGFM and per-device data storage are not in libfprint yet. There are no promises and no timelines.

## Acknowledgements and licence

- SIGFM's matcher code is copyright 2022 Matthieu Charette, Natasha England-Elbro and Timur Mangliev.
- Natalie Klestrup Röijezon first carried SIGFM toward libfprint, and Tooniis wrote MR !530.
- Sergey Subbotin's SIGFM branch (8 commits, with benchmark tooling copyright 2026) is the base of this fork.
- Omkar Chandorkar wrote the gallery fix.
- Raul Dias wrote the driver, the core fix and the replay test.
- The libfprint project and its contributors, for everything else.

Licence: LGPL-2.1-or-later, the same as upstream.

---

The original libfprint README follows.


<div align="center">

# LibFPrint

*LibFPrint is part of the **[FPrint][Website]** project.*

<br/>

[![Button Website]][Website]
[![Button Documentation]][Documentation]

[![Button Supported]][Supported]
[![Button Unsupported]][Unsupported]

[![Button Contribute]][Contribute]
[![Button Contributors]][Contributors]

</div>

## History

**LibFPrint** was originally developed as part of an
academic project at the **[University Of Manchester]**.

It aimed to hide the differences between consumer
fingerprint scanners and provide a single uniform
API to application developers.

## Goal

The ultimate goal of the **FPrint** project is to make
fingerprint scanners widely and easily usable under
common Linux environments.

## License

`Section 6` of the license states that for compiled works that use
this library, such works must include **LibFPrint** copyright notices
alongside the copyright notices for the other parts of the work.

**LibFPrint** includes code from **NIST's** **[NBIS]** software distribution.

We include **Bozorth3** from the **[US Export Controlled]**
distribution, which we have determined to be fine
being shipped in an open source project.

## Get in *touch*

 - [IRC] - `#fprint` @ `irc.oftc.net`
 - [Matrix] - `#fprint:matrix.org` bridged to the IRC channel
 - [MailingList] - low traffic, not much used these days

<br/>

<div align="right">

[![Badge License]][License]

</div>


<!----------------------------------------------------------------------------->

[Documentation]: https://fprint.freedesktop.org/libfprint-dev/
[Contributors]: https://gitlab.freedesktop.org/libfprint/libfprint/-/graphs/master
[Unsupported]: https://gitlab.freedesktop.org/libfprint/wiki/-/wikis/Unsupported-Devices
[Supported]: https://fprint.freedesktop.org/supported-devices.html
[Website]: https://fprint.freedesktop.org/
[MailingList]: https://lists.freedesktop.org/mailman/listinfo/fprint
[IRC]: ircs://irc.oftc.net:6697/#fprint
[Matrix]: https://matrix.to/#/#fprint:matrix.org

[Contribute]: ./HACKING.md
[License]: ./COPYING

[University Of Manchester]: https://www.manchester.ac.uk/
[US Export Controlled]: https://fprint.freedesktop.org/us-export-control.html
[NBIS]: http://fingerprint.nist.gov/NBIS/index.html


<!---------------------------------[ Badges ]---------------------------------->

[Badge License]: https://img.shields.io/badge/License-LGPL2.1-015d93.svg?style=for-the-badge&labelColor=blue


<!---------------------------------[ Buttons ]--------------------------------->

[Button Documentation]: https://img.shields.io/badge/Documentation-04ACE6?style=for-the-badge&logoColor=white&logo=BookStack
[Button Contributors]: https://img.shields.io/badge/Contributors-FF4F8B?style=for-the-badge&logoColor=white&logo=ActiGraph
[Button Unsupported]: https://img.shields.io/badge/Unsupported_Devices-EF2D5E?style=for-the-badge&logoColor=white&logo=AdBlock
[Button Contribute]: https://img.shields.io/badge/Contribute-66459B?style=for-the-badge&logoColor=white&logo=Git
[Button Supported]: https://img.shields.io/badge/Supported_Devices-428813?style=for-the-badge&logoColor=white&logo=AdGuard
[Button Website]: https://img.shields.io/badge/Homepage-3B80AE?style=for-the-badge&logoColor=white&logo=freedesktopDotOrg
