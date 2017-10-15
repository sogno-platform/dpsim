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
add_library(dpsim STATIC ${SOURCES})

target_include_directories(dpsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Python module
set(Python_ADDITIONAL_VERSIONS 3.5 3.6 3.7)
find_package(PythonInterp 3.5)
find_package(PythonLibs 3.5)

if(PythonInterp_FOUND AND PythonLibs_FOUND)
	file(GLOB PYTHON_SOURCES Python/*.cpp)

	include_directories(${PYTHON_INCLUDE_DIRS})
	link_libraries(${PYTHON_LIBRARIES})

	ADD_CUSTOM_TARGET(link_dpsim ALL DEPENDS dpsim COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:dpsim> dpsim.${PYTHON_MODULE_SUFFIX})

	execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)

	install(FILES $<TARGET_FILE:dpsim> DESTINATION ${PYTHON_PKGPATH} RENAME dpsim.${PYTHON_MODULE_SUFFIX})
endif()

# CIM Parser
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../CIM-XML-Parser)
	set(USE_CIM_VERSION "IEC61970_16v29a")
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../CIM-XML-Parser)

	file(GLOB CIM_SOURCES CIM/*.cpp)

	link_libraries(${PYTHON_LIBRARIES})
	target_sources(dpsim ${CIM_SOURCES})
endif()

#install(TARGETS dpsim
#	RUNTIME DESTINATION bin
#	LIBRARY DESTINATION lib
#)