# Vendored fixes for the pinned ramulator2 checkout. Each patch is applied to the
# submodule working tree at configure time and is skipped when already present,
# so re-configuring, a fresh clone and `git submodule update` all converge on the
# same sources. Drop a patch here once upstream carries the equivalent fix.
set(ARCHES_RAMULATOR_PATCHES
  "${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-param-dependent-template-keyword.patch"
)

find_package(Git QUIET)
set(_arches_ramulator_src "${PROJECT_SOURCE_DIR}/external/ramulator2")

if(NOT EXISTS "${_arches_ramulator_src}/src/base/param.h")
  message(FATAL_ERROR
    "The ramulator2 submodule is empty. Run:\n"
    "  git submodule update --init --recursive")
endif()

foreach(_patch IN LISTS ARCHES_RAMULATOR_PATCHES)
  get_filename_component(_patch_name "${_patch}" NAME)
  if(NOT GIT_FOUND)
    message(WARNING
      "Arches: git not found, cannot apply ${_patch_name} to ramulator2. "
      "Apply it by hand if the build fails.")
    break()
  endif()

  # --reverse --check succeeds only when the patch is already in the tree.
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${_patch}"
    WORKING_DIRECTORY "${_arches_ramulator_src}"
    RESULT_VARIABLE _patch_applied
    OUTPUT_QUIET ERROR_QUIET)

  if(_patch_applied EQUAL 0)
    continue()
  endif()

  execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply "${_patch}"
    WORKING_DIRECTORY "${_arches_ramulator_src}"
    RESULT_VARIABLE _patch_result
    ERROR_VARIABLE _patch_error)

  if(_patch_result EQUAL 0)
    message(STATUS "Arches: applied ${_patch_name} to ramulator2")
  else()
    message(FATAL_ERROR
      "Arches: failed to apply ${_patch_name} to ramulator2.\n${_patch_error}")
  endif()
endforeach()

set(ARCHES_RAMULATOR_SOURCES
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/base/factory.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/base/logging.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/base/request.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/base/stats.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/base/utils.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/frontend/impl/external_wrapper/gem5_frontend.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/translation/impl/no_translation.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/memory_system/impl/generic_DRAM_system.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/dram_controller/impl/rowpolicy/basic_rowpolicies.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/dram_controller/impl/scheduler/generic_scheduler.cpp"
  "${PROJECT_SOURCE_DIR}/external/ramulator2/src/dram_controller/impl/refresh/all_bank_refresh.cpp"
  "${PROJECT_SOURCE_DIR}/src/arches-v2/units/ramulator/config.cpp"
  "${PROJECT_SOURCE_DIR}/src/arches-v2/units/ramulator/unit-addr-mapper.cpp"
  "${PROJECT_SOURCE_DIR}/src/arches-v2/units/ramulator/unit-GDDR6.cpp"
  "${PROJECT_SOURCE_DIR}/src/arches-v2/units/ramulator/unit-generic-dram-controller.cpp"
)

# Ramulator implementations self-register through translation-unit static initializers.
# Use an object library so the linker cannot drop registration-only objects from
# a static archive before Factory::create_* looks them up at runtime.
add_library(arches_ramulator OBJECT ${ARCHES_RAMULATOR_SOURCES})
add_library(arches::ramulator ALIAS arches_ramulator)

target_compile_features(arches_ramulator PUBLIC cxx_std_20)

target_include_directories(arches_ramulator
  PUBLIC
    "${PROJECT_SOURCE_DIR}/external"
    "${PROJECT_SOURCE_DIR}/external/ramulator2/src"
)

target_link_libraries(arches_ramulator
  PUBLIC
    ${ARCHES_YAML_CPP_TARGET}
    ${ARCHES_SPDLOG_TARGET}
)

set_target_properties(arches_ramulator PROPERTIES FOLDER "Libraries")
source_group(TREE "${PROJECT_SOURCE_DIR}" FILES ${ARCHES_RAMULATOR_SOURCES})
