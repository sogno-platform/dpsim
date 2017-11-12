option(BUILD_SHARED_LIBS "Build shared library" OFF)

if (NOT WIN32)
	# needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "-fPIC")
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif()

if(WIN32)
	set(PYTHON_MODULE_SUFFIX "pyd")
	set(ARABICA_XML_BACKEND USE_MSXML)
elseif(LINUX)
	set(PYTHON_MODULE_SUFFIX "so")
	set(LIBS "-lrt -lvillas-ext")
else(APPLE)
	set(PYTHON_MODULE_SUFFIX "so")
endif()

file(GLOB SOURCES *.cpp Components/*.cpp)
add_library(dpsim ${SOURCES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Python module
set(Python_ADDITIONAL_VERSIONS 3.5 3.6 3.7)
find_package(PythonInterp 3.5)
find_package(PythonLibs 3.5)


if(PythonInterp_FOUND AND PythonLibs_FOUND)
	file(GLOB PYTHON_SOURCES Python/*.cpp)
	target_sources(dpsim PUBLIC ${PYTHON_SOURCES})

	target_include_directories(dpsim PUBLIC ${PYTHON_INCLUDE_DIRS})
	target_link_libraries(dpsim ${PYTHON_LIBRARIES})

	get_target_property(DPSIM_TYPE dpsim TYPE)
	if(${DPSIM_TYPE} STREQUAL SHARED_LIBRARY)
		add_custom_target(link_dpsim ALL DEPENDS dpsim COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:dpsim> dpsim.${PYTHON_MODULE_SUFFIX})

		execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)

		install(FILES $<TARGET_FILE:dpsim> DESTINATION ${PYTHON_PKGPATH} RENAME dpsim.${PYTHON_MODULE_SUFFIX})
	else()
		message(WARNING "In order to build DPsim as a Python module you must build it as a shared library: cmake -DBUILD_SHARED_LIBS=ON")
	endif()
endif()

# CIMpp
find_package(CIMpp)
if(CIMpp_FOUND)
	file(GLOB CIM_SOURCES CIM/*.cpp)
	target_sources(dpsim PUBLIC ${CIM_SOURCES})

	target_include_directories(dpsim PUBLIC ${CIMPP_INCLUDE_DIRS})
	target_link_libraries(dpsim ${CIMPP_LIBRARIES})
endif()
