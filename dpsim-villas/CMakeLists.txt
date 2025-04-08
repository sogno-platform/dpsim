if(WITH_PYBIND)
	pybind11_add_module(dpsimpyvillas src/pybind-dpsim-villas.cpp)
	set_target_properties(dpsimpyvillas
		PROPERTIES
		LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
	)
	target_link_libraries(dpsimpyvillas PRIVATE dpsim-villas)

	install(TARGETS dpsimpyvillas
		EXPORT dpsimpyvillas
		INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_PYTHON_SITEPACKAGESDIR}
	)
endif()

add_subdirectory(src)

if(DPSIM_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()

file(GLOB_RECURSE HEADER_FILES include/*.h)

target_sources(dpsim-villas PUBLIC
	FILE_SET public_headers
	TYPE HEADERS
	BASE_DIRS include
	FILES "${HEADER_FILES}"
)

install(TARGETS dpsim-villas
	EXPORT dpsim-villas
	FILE_SET public_headers DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
