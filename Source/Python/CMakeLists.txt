execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'), end='')" OUTPUT_VARIABLE PYTHON_EXT_SUFFIX)

add_library(dpsim_python SHARED
	Module.cpp
	Simulation.cpp
)

if(APPLE)
	set(INSTALL_RPATH "@loader_path")
else(UNIX)
	set(INSTALL_RPATH "\$ORIGIN")
endif()

target_link_libraries(dpsim_python dpsim cps_python cps ${LIBRARIES})
target_include_directories(dpsim_python PRIVATE ${INCLUDE_DIRS})

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