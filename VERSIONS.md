# STEPFOC versions

Which firmware your STEPFOC runs, and which release, bootloader and manual go with it.

## Releases

| Release | Date | What it is | Bootloader | Files |
|---|---|---|---|---|
| **V108** | 2026-08-26 | fixes `#Cal` failing with *Phase order is wrong!* on healthy motors | required | [download](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V108) |
| V107 | 2026-08-24 | fixes the KD term of PD mode over CAN; brake or coast over CAN | required | [download](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V107) |
| V106 | 2026-07-05 | no dropped CAN bursts, a watchdog, a CRC-checked configuration | required | [download](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V106) |
| V105 | 2026-07-04 | the first for the bootloader; reversed phase wiring calibrates | required | [download](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V105) |
| V103 | 2025-12-28 | the first stable public release, the last from `main` | none | [tag](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V103) |
| V102 | 2025-12-28 | the first official public release | none | [tag](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V102) |
| V101 | 2025-09-29 | the first release, based on the Spectral Micro firmware | none | [tag](https://github.com/Source-Robotics/STEPFOC-stepper-controller/releases/tag/V101) |

Releases from V105 on are built from the `bootloader-version` branch and carry the image
`stepfoc-firmware-V<number>.bin` with its `firmware.json`; the release notes are the changelog.

The manual has a version for [108](https://docs.source-robotics.com/STEPFOC-docs/latest/) (latest) and one for
[103](https://docs.source-robotics.com/STEPFOC-docs/103/); 105 to 107 have none of their own. The selector in the
manual's header switches between them.

## Which version do I have?

| | What differs | How to tell |
|---|---|---|
| Firmware 105 or newer | has the bootloader: updates over UART or CAN, with Motor GUI | `#Info` prints `Software version: <number>`; Motor GUI shows it in the driver's identity |
| Firmware 103 or older | no bootloader: [install it once](https://docs.source-robotics.com/STEPFOC-docs/latest/07-firmware/bootloader/) with a programming adapter | `#Info` too, but before 105 the number comes from the last `#Default`: 101 and 102 write 101, 103 writes 103 |
| A driver that reports 103 | some were shipped with the bootloader before 105, most likely with 103 built for it | with the bootloader the LED blinks for about two seconds at power-up before the firmware starts ([more](https://docs.source-robotics.com/STEPFOC-docs/latest/07-firmware/releases/#drivers-that-report-103)) |
| Board revision | v1, the only revision so far | `#Info` prints `Hardware version` |

## What works together

The bootloader, the Python library, the PAR6 runtime, the board revision and the matching Spectral Micro release for
each firmware release:
[Versions and compatibility](https://docs.source-robotics.com/STEPFOC-docs/latest/appendix/compatibility/) in the manual.
