cmake_minimum_required(VERSION 3.5)
project(DPsimExample)

include_directories(${EIGEN3_INCLUDE_DIR} "../../Source")

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

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# add files
set(SOURCES
		DPsimExample.cpp
		ReferenceCircuits.cpp
		SimpleCircuitTest.cpp
		SyncGenUnitTestVBR.cpp
		SynchronGenUnitTest.cpp
)

# CIMpp
find_package(CIMpp)
if(CIMpp_FOUND)
	include_directories(${CIMPP_INCLUDE_DIRS})
	link_libraries(${CIMPP_LIBRARIES})

	list(APPEND SOURCES
		CIMTest.cpp
	)
endif()

if(LINUX)
	list(APPEND SOURCES
		ShmemTest.cpp
	)
endif()

# targets
add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(DPsimExample dpsim)
