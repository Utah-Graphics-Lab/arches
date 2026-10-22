# Arches Startup Guide

This guide assumes a fresh clone and a first-time build for the ray tracing hardware course workflow.

## 1. Clone

```bash
git clone --recursive <arches-repo-url> arches
cd arches
```

If the repo was cloned without submodules:

```bash
git submodule update --init --recursive
```

Arches expects Ramulator2 at:

```text
external/ramulator2
```

Do not edit Ramulator2's CMake files for the Arches build. Arches wraps the required Ramulator sources from its own CMake.

## 2. Datasets

Place datasets in:

```text
datasets/
```

The default build config points `ARCHES_DATASET_DIR` at this folder. You can override it while configuring:

```bash
-DARCHES_DATASET_DIR=/path/to/arches/datasets
```

You can also override it at runtime:

```bash
--dataset-dir=/path/to/arches/datasets
```

## 3. Dependencies

Required packages:

- CMake 3.21 or newer
- C++20 compiler
- TBB
- yaml-cpp
- spdlog

Ramulator2 is a source submodule. It is not installed as a package.

### vcpkg

The repo has a `vcpkg.json` manifest. When you configure with the vcpkg toolchain file, vcpkg restores the manifest dependencies into a local `vcpkg_installed/` tree. You do not need a separate `vcpkg install` step unless you want to prefetch dependencies before configuring.

Windows example:

```powershell
git clone https://github.com/microsoft/vcpkg C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat
C:\dev\vcpkg\vcpkg.exe install --triplet x64-windows
```

Linux example:

```bash
git clone https://github.com/microsoft/vcpkg ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh
~/vcpkg/vcpkg install --triplet x64-linux
```

Then point CMake at the vcpkg toolchain. Either set the `VCPKG_ROOT` environment
variable to your vcpkg checkout (Arches auto-detects it during configure), or pass
the toolchain explicitly:

```bash
-DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

An explicit `-DCMAKE_TOOLCHAIN_FILE` always takes precedence over `VCPKG_ROOT`.

### Windows

Install Visual Studio 2022 with C++ development tools.

Install or use an existing vcpkg checkout. You can either set the `VCPKG_ROOT`
environment variable (Arches auto-detects it during configure) or pass the
toolchain path explicitly. An explicit flag always wins.

> **Note:** Visual Studio Developer prompts set `VCPKG_ROOT` to the vcpkg copy
> bundled with Visual Studio (`...\VC\vcpkg`). That copy is a frozen snapshot
> and cannot resolve this project's manifest, so Arches ignores it on purpose.
> Use a standalone clone of [microsoft/vcpkg](https://github.com/microsoft/vcpkg)
> as shown above. If a configure already failed with the bundled copy, delete
> the `build/windows` folder (or configure once with `--fresh`) so the cached
> toolchain path is cleared.

Example (explicit toolchain):

```powershell
cmake --preset windows -DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake
```

Example (auto-detected from `VCPKG_ROOT`):

```powershell
$env:VCPKG_ROOT = "C:/dev/vcpkg"
cmake --preset windows
```

### Linux

Use system packages if they provide CMake config packages for TBB, yaml-cpp, and spdlog. These commands are a bootstrap step before CMake configure; CMake does not run `apt`, `dnf`, or `pacman` for you. Otherwise use vcpkg explicitly, the same way as Windows.

Ubuntu/Debian:

```bash
sudo apt update
sudo apt install build-essential cmake git pkg-config libtbb-dev libyaml-cpp-dev libspdlog-dev
```

Fedora:

```bash
sudo dnf install gcc-c++ cmake git make pkgconf-pkg-config tbb-devel yaml-cpp-devel spdlog-devel
```

Arch:

```bash
sudo pacman -S --needed base-devel cmake git onetbb yaml-cpp spdlog
```

If your distro packages do not provide CMake package config files that satisfy `find_package`, use the vcpkg path instead:

```bash
make release CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
```

## 4. Configure and Build

### Windows Visual Studio

Configure once:

```powershell
cmake --preset windows
```

The solution is generated at:

```text
build/windows/arches.sln
```

Open the solution and pick `Debug` or `Release` in Visual Studio; the
configuration is chosen at build time, not at configure time. To build from
the command line instead:

```powershell
cmake --build build/windows --config Debug
cmake --build build/windows --config Release
```

### Linux

The root `makefile` drives CMake; the build mode is an argument:

```bash
make debug
make release   # also the default for plain `make`
```

Extra CMake options pass through `CMAKE_ARGS`:

```bash
make release CMAKE_ARGS="-DARCHES_DATASET_DIR=/path/to/datasets"
```

## 5. Runtime Layout

Build outputs are staged into:

```text
build/debug/
build/release/
```

Expected contents:

- `arches.exe` / `arches`
- `trax.exe` / `trax`
- required Windows runtime DLLs from package dependencies
- `config-files/`, copied from the common source config folder
- `kernels/trax/kernel` when an existing or built RISC-V kernel is available

CMake build trees (caches, object files, Visual Studio projects) live in
per-platform directories:

```text
build/windows/
build/linux-debug/
build/linux-release/
```

## 6. Run Smoke Tests

### Windows

```powershell
.\build\debug\arches.exe --scene-name=cornell-box --framebuffer-width=16 --framebuffer-height=16
.\build\debug\trax.exe --scene-name=sponza --framebuffer-width=16 --framebuffer-height=16
```

### Linux

```bash
./build/debug/arches --scene-name=cornell-box --framebuffer-width=16 --framebuffer-height=16
./build/debug/trax --scene-name=sponza --framebuffer-width=16 --framebuffer-height=16
```

## 7. Runtime Overrides

Useful runtime flags:

```text
--dataset-dir=<path>
--config-dir=<path>
--kernel-path=<path>
--scene-name=<name>
--framebuffer-width=<pixels>
--framebuffer-height=<pixels>
```

Runtime lookup order:

1. Command-line override.
2. Files staged next to `arches`.
3. Source-tree fallback.

## 8. RISC-V Kernel

The TRaX RISC-V kernel targets Arches' custom ISA and is built outside of
CMake with its own toolchain, using the makefile in `src/trax-kernel`:

```bash
cd src/trax-kernel
make riscv-dissasm
```

CMake never requires a RISC-V toolchain. The prebuilt kernel checked into the
source tree is staged next to the executables automatically:

```text
src/trax-kernel/riscv/kernel  ->  build/<config>/kernels/trax/kernel
```

## 9. Optional Standalone Kernel Tools

The default solution keeps only the course-critical targets enabled. To also build the older standalone STRaTA and dual-streaming native tools:

```powershell
cmake --preset windows -DARCHES_BUILD_LEGACY_KERNEL_TOOLS=ON
```

Or add the same option to a Linux build:

```bash
make debug CMAKE_ARGS="-DARCHES_BUILD_LEGACY_KERNEL_TOOLS=ON"
```

## 10. Benchmark Script

After building Release:

```bash
python scripts/benchmark.py --configuration Release
```

Override datasets if needed:

```bash
python scripts/benchmark.py --configuration Release --dataset-dir /path/to/datasets
```
