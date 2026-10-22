# Linux build wrapper for Arches.
#
#   make            -> release build
#   make debug      -> debug build
#   make release    -> release build
#   make clean      -> remove the Linux CMake build trees
#
# Executables and runtime assets are staged by CMake into build/debug and
# build/release. Extra CMake configure options can be passed via CMAKE_ARGS:
#
#   make release CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg.cmake"
#
# Windows does not use this file: run `cmake --preset windows` once and pick
# Debug/Release inside Visual Studio (see STARTUP_GUIDE.md).

BUILD_DIR_DEBUG   := build/linux-debug
BUILD_DIR_RELEASE := build/linux-release
JOBS ?= $(shell nproc 2>/dev/null || echo 4)
CMAKE_ARGS ?=

.PHONY: all debug release clean

all: release

debug:
	cmake -S . -B $(BUILD_DIR_DEBUG) -DCMAKE_BUILD_TYPE=Debug $(CMAKE_ARGS)
	cmake --build $(BUILD_DIR_DEBUG) -j$(JOBS)

release:
	cmake -S . -B $(BUILD_DIR_RELEASE) -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGS)
	cmake --build $(BUILD_DIR_RELEASE) -j$(JOBS)

clean:
	rm -rf $(BUILD_DIR_DEBUG) $(BUILD_DIR_RELEASE)
