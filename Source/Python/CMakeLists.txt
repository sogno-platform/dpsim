execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'), end='')" OUTPUT_VARIABLE PYTHON_EXT_SUFFIX)

if(NOT WIN32)
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif()

add_library(dpsim_python SHARED	
	Module.cpp
	Simulation.cpp

	Component.cpp
	Node.cpp
	Interface.cpp
	LoadCim.cpp
	SystemTopology.cpp
	
	Components/LinePi.cpp
	Components/LoadPQ.cpp
	
	Components/VoltageSourceNorton.cpp
	
	Components/CurrentSource.cpp
	Components/VoltageSource.cpp
	
	Components/Resistor.cpp
	Components/Inductor.cpp
	Components/Capacitor.cpp

	Utils.cpp
)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/Include/dpsim/Python)
add_custom_target(dpsim_python_list
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Scripts/find_cps_components.py
		${CMAKE_SOURCE_DIR}/Dependencies/libcps/Include/cps
		${CMAKE_BINARY_DIR}/Include/dpsim/Python/ComponentConstructors.h
)
add_dependencies(dpsim_python dpsim_python_list)

if(APPLE)
	set(INSTALL_RPATH "@loader_path")
else(UNIX)
	set(INSTALL_RPATH "\$ORIGIN")
endif()

target_link_libraries(dpsim_python dpsim cps ${LIBRARIES})
target_include_directories(dpsim_python PRIVATE ${INCLUDE_DIRS})
target_compile_options(dpsim_python PUBLIC ${DPSIM_CXX_FLAGS})

set_target_properties(dpsim_python
	PROPERTIES
		OUTPUT_NAME "_dpsim"
		SUFFIX ${PYTHON_EXT_SUFFIX}
		PREFIX ""
		INSTALL_RPATH ${INSTALL_RPATH}
)

# Install native extension
install(
	TARGETS dpsim_python
	DESTINATION ${PYTHON_PKGPATH}
)

# Install Python wrapper code
install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dpsim
	DESTINATION ${PYTHON_PKGPATH}
	PATTERN *.py
)