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
