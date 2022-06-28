if(WITH_PYBIND)
    pybind11_add_module(dpsimpyvillas src/pybind-dpsim-villas.cpp)
    target_link_libraries(dpsimpyvillas PRIVATE dpsim-villas)
endif()

add_subdirectory(src)

if(DPSIM_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()