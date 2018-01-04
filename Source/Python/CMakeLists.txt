include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/..)

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'), end='')" OUTPUT_VARIABLE PYTHON_EXT_SUFFIX)

add_library(dpsim-python SHARED
	Component.cpp
	Module.cpp
	Simulation.cpp
	Interface.cpp
)

if(WITH_CIM)
	list(APPEND PYTHON_SOURCES
		Python/LoadCim.cpp
	)
endif(WITH_CIM)

target_sources(dpsim-python PRIVATE ${PYTHON_SOURCES})
target_link_libraries(dpsim-python dpsim ${PYTHON_LIBRARIES})
target_include_directories(dpsim-python PRIVATE ${PYTHON_INCLUDE_DIRS} ${INCLUDE_DIRS})
set_target_properties(dpsim-python
	PROPERTIES
		LIBRARY_OUTPUT_NAME "_dpsim"
		SUFFIX ${PYTHON_EXT_SUFFIX}
		PREFIX ""
)

# Install native extension
install(
	TARGETS dpsim-python
	DESTINATION ${PYTHON_PKGPATH}
)

# Install Python wrapper code
install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dpsim
	DESTINATION ${PYTHON_PKGPATH}
	PATTERN *.py
)