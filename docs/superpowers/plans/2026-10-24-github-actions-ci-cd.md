# GitHub Actions CI/CD Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add GitHub Actions CI for the RS485/CAN Debug/Release firmware matrix and publish Release artifacts when a `v*` tag is pushed.

**Architecture:** One workflow at `.github/workflows/firmware-ci.yml` runs the existing build wrapper in a four-entry matrix, validates output files, and uploads each matrix result. A dependent job runs only for version tags, downloads the two Release matrix artifacts, renames them with backend/configuration names, and creates a GitHub Release with generated notes using the runner-provided `GITHUB_TOKEN`.

**Tech Stack:** GitHub Actions, Ubuntu latest, CMake, Ninja, GNU Arm Embedded GCC/binutils, GitHub CLI (`gh`).

---

### Task 1: Add the firmware CI matrix workflow

**Files:**
- Create: `.github/workflows/firmware-ci.yml`

- [x] **Step 1: Define triggers and least-privilege default permissions**

Configure `pull_request`, branch `push`, and `v*` tag `push` triggers. Set the
workflow default to `contents: read`; do not grant write permissions to build
matrix jobs.

- [x] **Step 2: Define the four backend/configuration combinations**

Use a matrix with `RS485` and `CAN` crossed with `Debug` and `Release`. Use
`ubuntu-latest`, disable fail-fast so all combinations report status, and set
the job timeout to 20 minutes.

- [x] **Step 3: Install and validate the native build tools**

Check out the source, install `cmake`, `ninja-build`, `gcc-arm-none-eabi`, and
`binutils-arm-none-eabi` through apt, then run:

```bash
bash -n scripts/build.sh scripts/flash_openocd.sh scripts/flash_stlink.sh
```

- [x] **Step 4: Build and verify each matrix result**

Invoke:

```bash
scripts/build.sh \
  --backend "${{ matrix.backend }}" \
  --config "${{ matrix.config }}" \
  --full --jobs 2
```

Verify these files exist under `build/${{ matrix.backend }}-${{ matrix.config }}/`:

```text
safeMRC_for_DM_firmware.elf
safeMRC_for_DM_firmware.hex
safeMRC_for_DM_firmware.bin
safeMRC_for_DM_firmware.map
```

- [x] **Step 5: Upload each job's artifacts**

Use `actions/upload-artifact@v4` with the artifact name
`firmware-${{ matrix.backend }}-${{ matrix.config }}`, upload the four verified
files, and set retention to 14 days.

### Task 2: Add version-tag GitHub Release publishing

**Files:**
- Modify: `.github/workflows/firmware-ci.yml`

- [x] **Step 1: Gate the Release job on successful matrix builds and version tags**

Make the job depend on the complete matrix job and require both
`github.event_name == 'push'` and a `refs/tags/v*` ref. Grant this job only
`contents: write`; keep the workflow default read-only.

- [x] **Step 2: Download only the Release matrix artifacts**

Use `actions/download-artifact@v4` with the exact names
`firmware-RS485-Release` and `firmware-CAN-Release`, placing each artifact in a
separate directory below `release-assets/`.

- [x] **Step 3: Name and publish release files**

Rename each downloaded file to include backend and configuration, for example
`safeMRC-CAN-Release.hex`. Use the preinstalled `gh` command to run
`gh release create "$GITHUB_REF_NAME" --generate-notes release-assets/*/*`
with `GH_TOKEN: ${{ secrets.GITHUB_TOKEN }}`. Do not invoke either flash target.

### Task 3: Document and verify the workflow

**Files:**
- Modify: `README.md`
- Modify: `README_CN.md`
- Verify: `.github/workflows/firmware-ci.yml`

- [x] **Step 1: Document CI triggers, matrix builds, downloads, and tag release**

Add a short section to both READMEs explaining that PRs and branch pushes build
both backends in Debug/Release, that each run uploads temporary artifacts, and
that pushing a `v*` tag creates a Release with Release configuration outputs.
State that hosted CI does not flash physical hardware.

- [x] **Step 2: Validate workflow syntax and local consistency**

Run `actionlint .github/workflows/firmware-ci.yml` when `actionlint` is
installed. Also parse the YAML with an available YAML parser, check the shell
commands with `bash -n`, verify all artifact paths match `scripts/build.sh`,
and run `git diff --check` on changed files.

- [x] **Step 3: Verify the local build wrapper still succeeds**

Run:

```bash
scripts/build.sh --backend RS485 --config Debug --jobs 2
scripts/build.sh --backend CAN --config Release --jobs 2
```

Both commands must exit successfully and leave ELF, HEX, BIN, and MAP outputs in
their respective build directories. Document that the GitHub-hosted workflow
itself can only be fully confirmed after pushing the branch to GitHub.

### Execution status

The workflow and README documentation are implemented. Local validation passed
for Bash syntax, YAML parsing with PyYAML, matrix/release artifact references,
README workflow references, and `git diff --check`. `actionlint` was not
installed in this environment. The local RS485 Debug and CAN Release builds
completed successfully. GitHub-hosted execution and tag release publishing
still require pushing this branch to the repository.
