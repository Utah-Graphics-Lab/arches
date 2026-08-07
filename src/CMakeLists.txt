add_subdirectory("trax-kernel")
add_subdirectory("arches-v2")

if(ARCHES_BUILD_LEGACY_KERNEL_TOOLS)
  add_subdirectory("dual-streaming-kernel")
  add_subdirectory("strata-rt-kernel")
endif()
