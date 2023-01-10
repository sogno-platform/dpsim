configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/include/dpsim/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/include/dpsim/Config.h
)

set(DPSIM_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}/include
)

add_subdirectory(src)

if(WITH_PYBIND)
	set(DPSIMPY_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/Include/dpsim/pybind)
	add_subdirectory(src/pybind)
endif()

if(DPSIM_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()
