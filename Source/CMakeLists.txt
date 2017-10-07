cmake_minimum_required(VERSION 3.5)

if (NOT WIN32)
	# needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "-fPIC")
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif()

# enable C++11 support
set(CMAKE_CXX_STANDARD 11)

# library configuration etc.
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../Libraries/eigen)
	set(EIGEN3_INCLUDE_DIR ../Libraries/eigen)
else()
	find_package(Eigen3 REQUIRED)
endif()

set(Python_ADDITIONAL_VERSIONS 3.5 3.6 3.7)
find_package(PythonInterp 3.5)
find_package(PythonLibs 3.5)

if(WIN32)
	set(PYTHON_MODULE_SUFFIX "pyd")
	set(ARABICA_XML_BACKEND USE_MSXML)
elseif(LINUX)
	set(PYTHON_MODULE_SUFFIX "so")
	set(LIBS "-lrt -lvillas-ext")
else(APPLE)
	set(PYTHON_MODULE_SUFFIX "so")
endif()

include_directories(${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})

set(USE_CIM_VERSION "IEC61970_16v29a")
add_subdirectory(CIM-XML-Parser)

file(GLOB SOURCES *.cpp Components/*.cpp)
add_library(dpsim SHARED ${SOURCES})
target_link_libraries(dpsim CIMParser ${PYTHON_LIBRARIES} ${LIBS})
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)

ADD_CUSTOM_TARGET(link_dpsim ALL DEPENDS dpsim COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:dpsim> dpsim.${PYTHON_MODULE_SUFFIX})

install(FILES $<TARGET_FILE:dpsim> DESTINATION ${PYTHON_PKGPATH} RENAME dpsim.${PYTHON_MODULE_SUFFIX})
