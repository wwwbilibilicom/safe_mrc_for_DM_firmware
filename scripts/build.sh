#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: scripts/build.sh [options]

Options:
  -i, --incremental       Incremental build (default)
  -f, --full              Clean the selected build tree, then rebuild all files
  -b, --backend NAME      RS485 or CAN (default: RS485)
  -c, --config NAME       Debug, Release, RelWithDebInfo, or MinSizeRel
  -j, --jobs N            Number of parallel build jobs
  -h, --help              Show this help

Examples:
  scripts/build.sh
  scripts/build.sh --full
  scripts/build.sh --backend CAN --full
  scripts/build.sh --backend RS485 --config Release --jobs 8
EOF
}

mode=incremental
backend=RS485
config=Debug
jobs=""
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--incremental)
            mode=incremental
            shift
            ;;
        -f|--full)
            mode=full
            shift
            ;;
        -b|--backend)
            [[ $# -ge 2 ]] || { echo "error: --backend needs RS485 or CAN" >&2; exit 2; }
            backend=${2^^}
            shift 2
            ;;
        -c|--config)
            [[ $# -ge 2 ]] || { echo "error: --config needs a build type" >&2; exit 2; }
            config=$2
            shift 2
            ;;
        -j|--jobs)
            [[ $# -ge 2 ]] || { echo "error: --jobs needs a positive integer" >&2; exit 2; }
            jobs=$2
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

case "$backend" in
    RS485|CAN) ;;
    *) echo "error: backend must be RS485 or CAN, got '$backend'" >&2; exit 2 ;;
esac

case "$config" in
    Debug|Release|RelWithDebInfo|MinSizeRel) ;;
    *) echo "error: unsupported config '$config'" >&2; exit 2 ;;
esac

if [[ -n "$jobs" && ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: jobs must be a positive integer, got '$jobs'" >&2
    exit 2
fi

for tool in cmake ninja arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: required tool is not installed or not on PATH: $tool" >&2
        exit 127
    fi
done

build_dir="$project_root/build/${backend}-${config}"
toolchain_file="$project_root/cmake/gcc-arm-none-eabi.cmake"

echo "Build mode: $mode"
echo "Backend:    $backend"
echo "Config:     $config"
echo "Directory:  $build_dir"

if [[ ! -f "$build_dir/build.ninja" || "$mode" == full ]]; then
    cmake -S "$project_root" -B "$build_dir" -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE="$toolchain_file" \
        -DCMAKE_BUILD_TYPE="$config" \
        -DMRC_COM_BACKEND="$backend"
else
    echo "Using existing CMake configuration"
fi

if [[ "$mode" == full && -f "$build_dir/build.ninja" ]]; then
    cmake --build "$build_dir" --target clean
fi

build_args=(--build "$build_dir")
if [[ -n "$jobs" ]]; then
    build_args+=(--parallel "$jobs")
fi
cmake "${build_args[@]}"
cmake --build "$build_dir" --target size

echo "Build complete: $build_dir/safeMRC_for_DM_firmware.elf"
