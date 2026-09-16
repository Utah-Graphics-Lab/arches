cmake_minimum_required(VERSION 3.10)

project(arches-v2 LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Allow old dependencies (e.g. yaml-cpp 0.7.0) that declare
# cmake_minimum_required(VERSION < 3.5) to configure under CMake >= 4.
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)

if(MSVC)
  # fmt (bundled inside spdlog) requires UTF-8 encoded sources.
  add_compile_options(/utf-8)
endif()

# ramulator2 fetches spdlog v1.11.0, whose bundled fmt 9.1 uses
# stdext::checked_array_iterator - removed from the MSVC STL in toolset 14.51
# (VS 2026). FetchContent honours the first declaration of a dependency, so
# declaring spdlog here overrides ramulator2's and pins a release whose bundled
# fmt (11.x) builds with current MSVC.
include(FetchContent)
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.15.3
  SOURCE_DIR     ${CMAKE_SOURCE_DIR}/ext/spdlog
)

add_subdirectory(external)
add_subdirectory("src")
