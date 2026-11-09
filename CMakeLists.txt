cmake_minimum_required(VERSION 3.21)

# vcpkg toolchain auto-detection (must run before project())
# Precedence:
#   1. An explicit -DCMAKE_TOOLCHAIN_FILE=...
#   2. Otherwise, if the VCPKG_ROOT environment variable points at a vcpkg
#      checkout, its toolchain is used automatically
#   3. Otherwise nothing is forced and find_package() resolves system packages.
# This never *requires* vcpkg; it only removes friction when VCPKG_ROOT is set.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}" _arches_vcpkg_root)
  string(TOLOWER "${_arches_vcpkg_root}" _arches_vcpkg_root_lower)
  set(_arches_vcpkg_toolchain "${_arches_vcpkg_root}/scripts/buildsystems/vcpkg.cmake")
  if(_arches_vcpkg_root_lower MATCHES "microsoft visual studio")
    # Visual Studio Developer prompts set VCPKG_ROOT to the vcpkg copy bundled
    # with VS. That copy is a frozen snapshot without git history and cannot
    # reliably resolve this manifest, so it is skipped on purpose.
    message(STATUS
      "Arches: ignoring VCPKG_ROOT at '${_arches_vcpkg_root}' (vcpkg bundled with Visual Studio). "
      "Use a standalone vcpkg clone instead: set VCPKG_ROOT to it or pass "
      "-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake (see STARTUP_GUIDE.md).")
  elseif(EXISTS "${_arches_vcpkg_toolchain}")
    set(CMAKE_TOOLCHAIN_FILE "${_arches_vcpkg_toolchain}"
        CACHE FILEPATH "Vcpkg toolchain file (auto-detected from VCPKG_ROOT)")
    message(STATUS "Arches: using vcpkg toolchain from VCPKG_ROOT (${_arches_vcpkg_root})")
  endif()
  unset(_arches_vcpkg_root)
  unset(_arches_vcpkg_root_lower)
  unset(_arches_vcpkg_toolchain)
endif()

project(arches-v2 LANGUAGES CXX C)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Supported configurations" FORCE)
endif()

set(ARCHES_DATASET_DIR "${PROJECT_SOURCE_DIR}/datasets" CACHE PATH "Path to Arches datasets")
option(ARCHES_BUILD_LEGACY_KERNEL_TOOLS "Build standalone STRaTA and dual-streaming kernel tools" OFF)

set(ARCHES_RUNTIME_DEBUG_DIR "${PROJECT_SOURCE_DIR}/build/debug")
set(ARCHES_RUNTIME_RELEASE_DIR "${PROJECT_SOURCE_DIR}/build/release")
set(ARCHES_RAMULATOR_CONFIG_DIR "${PROJECT_SOURCE_DIR}/src/arches-v2/units/ramulator/config-files")

# The prebuilt kernel checked into the source tree is staged next to the executables.
set(ARCHES_DEFAULT_KERNEL_PATH "${PROJECT_SOURCE_DIR}/src/trax-kernel/riscv/kernel")

find_package(TBB CONFIG QUIET)
find_package(yaml-cpp CONFIG QUIET)
find_package(spdlog CONFIG QUIET)
find_package(Threads REQUIRED)

# Aggregate missing third-party dependencies into one actionable message
set(_arches_missing_deps "")
if(NOT TARGET TBB::tbb)
  list(APPEND _arches_missing_deps "TBB")
endif()
if(NOT TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
  list(APPEND _arches_missing_deps "yaml-cpp")
endif()
if(NOT TARGET spdlog::spdlog AND NOT TARGET spdlog)
  list(APPEND _arches_missing_deps "spdlog")
endif()

if(_arches_missing_deps)
  string(REPLACE ";" ", " _arches_missing_str "${_arches_missing_deps}")
  message(FATAL_ERROR
    "Arches could not find required dependencies: ${_arches_missing_str}.\n"
    "See STARTUP_GUIDE.md for setup. Quick options:\n"
    "  vcpkg:         set the VCPKG_ROOT environment variable,\n"
    "                 or pass -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake\n"
    "  Ubuntu/Debian: sudo apt install -y libtbb-dev libyaml-cpp-dev libspdlog-dev\n"
    "  Fedora:        sudo dnf install tbb-devel yaml-cpp-devel spdlog-devel\n"
    "  Arch:          sudo pacman -S onetbb yaml-cpp spdlog")
endif()

if(TARGET yaml-cpp::yaml-cpp)
  set(ARCHES_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(ARCHES_YAML_CPP_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp package was found, but no known imported target exists.")
endif()

if(TARGET spdlog::spdlog)
  set(ARCHES_SPDLOG_TARGET spdlog::spdlog)
elseif(TARGET spdlog)
  set(ARCHES_SPDLOG_TARGET spdlog)
else()
  message(FATAL_ERROR "spdlog package was found, but no known imported target exists.")
endif()

function(arches_set_output_dirs target_name)
  set_target_properties(${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${ARCHES_RUNTIME_DEBUG_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${ARCHES_RUNTIME_RELEASE_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
    PDB_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/pdb/Debug"
    PDB_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/pdb/Release"
    COMPILE_PDB_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/pdb/Debug"
    COMPILE_PDB_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/pdb/Release"
  )
endfunction()

function(arches_copy_runtime_dlls target_name)
  if(WIN32)
    set(copy_script "${CMAKE_CURRENT_BINARY_DIR}/${target_name}-copy-runtime-dlls-$<CONFIG>.cmake")
    file(GENERATE OUTPUT "${copy_script}" CONTENT
"set(_dlls \"$<TARGET_RUNTIME_DLLS:${target_name}>\")
foreach(_dll IN LISTS _dlls)
  if(EXISTS \"\${_dll}\")
    file(COPY \"\${_dll}\" DESTINATION \"$<TARGET_FILE_DIR:${target_name}>\")
  endif()
endforeach()
" TARGET ${target_name})
    add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -P "${copy_script}"
      VERBATIM
    )
  endif()
endfunction()

function(arches_stage_runtime_assets target_name)
  add_custom_command(TARGET ${target_name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/config-files"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${ARCHES_RAMULATOR_CONFIG_DIR}" "$<TARGET_FILE_DIR:${target_name}>/config-files"
    VERBATIM
  )

  if(EXISTS "${ARCHES_DEFAULT_KERNEL_PATH}")
    add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/kernels/trax"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${ARCHES_DEFAULT_KERNEL_PATH}" "$<TARGET_FILE_DIR:${target_name}>/kernels/trax/kernel"
      VERBATIM
    )
  endif()
endfunction()

add_subdirectory(external)
add_subdirectory("src")

if(TARGET arches)
  set_property(DIRECTORY "${PROJECT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT arches)
endif()
