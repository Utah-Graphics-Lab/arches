set(TRAX_SOURCES
  "main.cpp"
  "stbi.cpp"
)

set(TRAX_HEADERS
  "custom-instr.hpp"
  "include.hpp"
  "intersect.hpp"
  "stdafx.hpp"
)

add_executable(trax ${TRAX_SOURCES} ${TRAX_HEADERS})

set_target_properties(trax PROPERTIES
  OUTPUT_NAME "trax"
  FOLDER "Applications"
  VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:trax>"
  VS_DEBUGGER_COMMAND_ARGUMENTS "--scene-name=sponza --framebuffer-width=16 --framebuffer-height=16"
)

arches_set_output_dirs(trax)

target_compile_features(trax PRIVATE cxx_std_20)

file(TO_CMAKE_PATH "${ARCHES_DATASET_DIR}" ARCHES_DEFAULT_DATASET_DIR_CMAKE)
target_compile_definitions(trax
  PRIVATE
    ARCHES_DEFAULT_DATASET_DIR="${ARCHES_DEFAULT_DATASET_DIR_CMAKE}"
)

target_compile_options(trax
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
    $<$<AND:$<PLATFORM_ID:Linux>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-march=native>
)

target_include_directories(trax
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
)

# stdc++fs: std::filesystem lives in a separate library on GCC < 9.
target_link_libraries(trax
  PRIVATE
    Threads::Threads
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${TRAX_SOURCES} ${TRAX_HEADERS})
