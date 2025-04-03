set(DPSIMPY_SOURCES
	DPComponents.cpp
	EMTComponents.cpp
	SPComponents.cpp
	SignalComponents.cpp
	BaseComponents.cpp
	main.cpp
	Utils.cpp
	Attributes.cpp
)

pybind11_add_module(dpsimpy ${DPSIMPY_SOURCES})
set_target_properties(dpsimpy
	PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
target_link_libraries(dpsimpy PRIVATE dpsim)
target_include_directories(dpsimpy PUBLIC ${DPSIMPY_INCLUDE_DIRS})
add_dependencies(pybind dpsimpy)
