set(ARCHES_SOURCES
  "main.cpp"
  "isa/registers.cpp"
  "isa/riscv.cpp"
  "simulator/simulator.cpp"
  "units/trax/unit-rt-core.cpp"
  "units/unit-cache.cpp"
  "units/unit-cache-base.cpp"
  "units/unit-dram.cpp"
  "units/unit-stream-cache.cpp"
  "units/unit-texture.cpp"
  "units/unit-tp.cpp"
  "util/elf.cpp"
  "util/file.cpp"
  "util/stbi.cpp"
  "util/string.cpp"
)

file(GLOB_RECURSE ARCHES_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")
file(GLOB ARCHES_RAMULATOR_CONFIG_FILES CONFIGURE_DEPENDS "units/ramulator/config-files/*.yaml")

add_executable(arches
  ${ARCHES_SOURCES}
  ${ARCHES_HEADERS}
  ${ARCHES_RAMULATOR_CONFIG_FILES}
)

set_target_properties(arches PROPERTIES
  OUTPUT_NAME "arches"
  FOLDER "Applications"
  VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:arches>"
  VS_DEBUGGER_COMMAND_ARGUMENTS "--scene-name=cornell-box --framebuffer-width=16 --framebuffer-height=16"
)

arches_set_output_dirs(arches)

target_compile_features(arches PRIVATE cxx_std_20)

file(TO_CMAKE_PATH "${PROJECT_SOURCE_DIR}" ARCHES_PROJECT_SOURCE_DIR_CMAKE)
file(TO_CMAKE_PATH "${ARCHES_DATASET_DIR}" ARCHES_DEFAULT_DATASET_DIR_CMAKE)
file(TO_CMAKE_PATH "${ARCHES_RAMULATOR_CONFIG_DIR}" ARCHES_RAMULATOR_CONFIG_DIR_CMAKE)
file(TO_CMAKE_PATH "${ARCHES_DEFAULT_KERNEL_PATH}" ARCHES_DEFAULT_KERNEL_PATH_CMAKE)

target_compile_definitions(arches
  PRIVATE
    UNICODE
    _UNICODE
    ARCHES_PROJECT_SOURCE_DIR="${ARCHES_PROJECT_SOURCE_DIR_CMAKE}"
    ARCHES_DEFAULT_DATASET_DIR="${ARCHES_DEFAULT_DATASET_DIR_CMAKE}"
    ARCHES_DEFAULT_CONFIG_DIR="${ARCHES_RAMULATOR_CONFIG_DIR_CMAKE}"
    ARCHES_DEFAULT_KERNEL_PATH="${ARCHES_DEFAULT_KERNEL_PATH_CMAKE}"
)

target_compile_options(arches
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
    $<$<AND:$<PLATFORM_ID:Linux>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-march=native>
)

target_include_directories(arches
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/src"
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/external"
)

# stdc++fs: std::filesystem lives in a separate library on GCC < 9.
target_link_libraries(arches
  PRIVATE
    TBB::tbb
    Threads::Threads
    arches::ramulator
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
)

arches_stage_runtime_assets(arches)
arches_copy_runtime_dlls(arches)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${ARCHES_SOURCES} ${ARCHES_HEADERS} ${ARCHES_RAMULATOR_CONFIG_FILES})
