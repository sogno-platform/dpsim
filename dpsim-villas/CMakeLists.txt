if(WITH_PYBIND)
	pybind11_add_module(dpsimpyvillas src/pybind-dpsim-villas.cpp)
	set_target_properties(dpsimpyvillas
		PROPERTIES
		LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
	)
	target_link_libraries(dpsimpyvillas PRIVATE dpsim-villas)
endif()

add_subdirectory(src)

if(DPSIM_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()
