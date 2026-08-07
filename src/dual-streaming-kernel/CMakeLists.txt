set(DUAL_STREAMING_KERNEL_SOURCES
  "main.cpp"
)

set(DUAL_STREAMING_KERNEL_HEADERS
  "custom-instr.hpp"
  "include.hpp"
  "intersect.hpp"
  "stdafx.hpp"
  "work-item.hpp"
)

add_executable(dual_streaming_kernel
  ${DUAL_STREAMING_KERNEL_SOURCES}
  ${DUAL_STREAMING_KERNEL_HEADERS}
)

set_target_properties(dual_streaming_kernel PROPERTIES
  OUTPUT_NAME "dual-streaming-kernel"
  FOLDER "Optional Kernels"
  VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:dual_streaming_kernel>"
)

arches_set_output_dirs(dual_streaming_kernel)

target_compile_features(dual_streaming_kernel PRIVATE cxx_std_20)

target_compile_options(dual_streaming_kernel
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
    $<$<AND:$<PLATFORM_ID:Linux>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-march=native>
)

target_include_directories(dual_streaming_kernel
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(dual_streaming_kernel
  PRIVATE
    Threads::Threads
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/riscv")
  add_custom_command(TARGET dual_streaming_kernel POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/riscv" "$<TARGET_FILE_DIR:dual_streaming_kernel>/riscv"
    VERBATIM
  )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${DUAL_STREAMING_KERNEL_SOURCES} ${DUAL_STREAMING_KERNEL_HEADERS})
