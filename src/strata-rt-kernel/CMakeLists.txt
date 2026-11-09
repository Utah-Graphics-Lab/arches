set(STRATA_RT_KERNEL_SOURCES
  "main.cpp"
)

set(STRATA_RT_KERNEL_HEADERS
  "custom-instr.hpp"
  "include.hpp"
  "intersect.hpp"
  "ray-data.hpp"
  "stdafx.hpp"
)

add_executable(strata_rt_kernel
  ${STRATA_RT_KERNEL_SOURCES}
  ${STRATA_RT_KERNEL_HEADERS}
)

set_target_properties(strata_rt_kernel PROPERTIES
  OUTPUT_NAME "strata-rt-kernel"
  FOLDER "Optional Kernels"
  VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:strata_rt_kernel>"
)

arches_set_output_dirs(strata_rt_kernel)

target_compile_features(strata_rt_kernel PRIVATE cxx_std_20)

target_compile_options(strata_rt_kernel
  PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
    $<$<AND:$<PLATFORM_ID:Linux>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-march=native>
)

target_include_directories(strata_rt_kernel
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(strata_rt_kernel
  PRIVATE
    Threads::Threads
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/riscv")
  add_custom_command(TARGET strata_rt_kernel POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/riscv" "$<TARGET_FILE_DIR:strata_rt_kernel>/riscv"
    VERBATIM
  )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${STRATA_RT_KERNEL_SOURCES} ${STRATA_RT_KERNEL_HEADERS})
