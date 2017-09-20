cmake_minimum_required(VERSION 3.0)
project(DPsim)

# needed so CIMParser and arabica (which are first compiled into static libs)
# can be included in the .so
set(CMAKE_CXX_FLAGS "-fPIC")

# library configuration etc.
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../Libraries/eigen)
	set(EIGEN3_INCLUDE_DIR ../Libraries/eigen)
else()
	find_package(Eigen3 REQUIRED)
endif()

find_package(PythonLibs)

if(WIN32)
	set(MODNAME "dpsim.dll")
	set(ARABICA_XML_BACKEND USE_MSXML)
else()
	set(MODNAME "dpsim.so")
	set(LIBS "-lrt -lvillas-ext")
endif()

include_directories(${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
add_subdirectory(CIM-XML-Parser)

file(GLOB SOURCES *.cpp Components/*.cpp)
add_library(dpsim SHARED ${SOURCES})
target_link_libraries(dpsim CIMParser pthread ${PYTHON_LIBRARIES} ${LIBS})
execute_process(COMMAND python -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)
install(FILES build/libdpsim.so DESTINATION ${PYTHON_PKGPATH} RENAME ${MODNAME})

