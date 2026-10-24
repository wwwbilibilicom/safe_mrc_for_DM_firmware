# GitHub Actions CI/CD Design

## Goal

Add an automated GitHub Actions workflow that builds the STM32H723 firmware for
both communication backends on pull requests and pushes, then publishes
versioned build artifacts as a GitHub Release when a version tag is pushed.
The workflow reuses the repository's `scripts/build.sh` so local and hosted
builds follow the same CMake/Ninja path.

## Triggers

- `pull_request`: run firmware builds and script checks for proposed changes.
- `push` to branches: run the same CI checks after changes land on a branch.
- `push` of tags matching `v*`: build all variants and create a GitHub Release.
- Changes limited to documentation may still run CI; path filtering is omitted
  initially to keep workflow behavior simple and predictable.

## Build matrix

Run four independent jobs on `ubuntu-latest`:

| Backend | Configuration |
| --- | --- |
| RS485 | Debug |
| RS485 | Release |
| CAN | Debug |
| CAN | Release |

Install CMake, Ninja, and the GNU Arm Embedded GCC/binutils packages available
on the runner. Check shell syntax for `scripts/build.sh`,
`scripts/flash_openocd.sh`, and `scripts/flash_stlink.sh`, then invoke
`scripts/build.sh` with the matrix backend and configuration. Each job verifies
that `.elf`, `.hex`, `.bin`, and `.map` outputs exist.

## Artifacts

Upload all four files from each matrix job as a uniquely named artifact:

```text
firmware-RS485-Debug
firmware-RS485-Release
firmware-CAN-Debug
firmware-CAN-Release
```

Use a finite retention period suitable for CI artifacts. Artifacts are build
outputs for inspection and download; they are not committed to the repository.

## Release behavior

For a pushed tag matching `v*`, wait for all matrix build jobs to succeed, then
download the Release-configuration artifacts and publish a GitHub Release
using the pushed tag. Attach ELF, HEX, BIN, and MAP files with backend and
configuration included in each filename. Enable generated release notes.

The release job requires only the repository's provided `GITHUB_TOKEN` with
contents-write permission. Do not add long-lived secrets or third-party
credentials. PR jobs must not receive write permissions.

## Hardware boundary

Hosted jobs do not invoke `flash` or `flash-stlink`: standard GitHub-hosted
runners have no project probe or target board attached. Hardware flashing and
peripheral smoke tests remain local/manual. A self-hosted runner workflow can
be designed separately if a permanently connected and controlled development
board becomes available.

## Acceptance criteria

1. Pull requests and branch pushes build all four backend/configuration
   combinations and report failures through the workflow status.
2. Each successful matrix job uploads the expected ELF, HEX, BIN, and MAP
   artifacts under a unique name.
3. A `v*` tag creates one release only after all four builds pass and attaches
   the Release artifacts for both communication backends.
4. The workflow invokes the existing build wrapper and performs no hardware
   flashing.
5. Workflow permissions follow least privilege: read-only for CI and
   contents-write only for the tag release job.

## Out of scope

- Flashing a physical board from hosted CI.
- Running a self-hosted runner or hardware-in-the-loop tests.
- Building Keil/MDK artifacts.
- Publishing package registries or Docker images.
