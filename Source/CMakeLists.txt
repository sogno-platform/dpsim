cmake_minimum_required(VERSION 3.0)
project(DPsim)

if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../Libraries/eigen)
	set(EIGEN3_INCLUDE_DIR ../Libraries/eigen)
else()
	find_package(Eigen3 REQUIRED)
endif()

if(WIN32)
	set(ARABICA_XML_BACKEND USE_MSXML)
else()
	set(LIBS "-lrt -lvillas-ext")
endif()

add_subdirectory(CIM-XML-Parser)

file(GLOB SOURCES *.cpp Components/*.cpp Examples/*.cpp)

add_executable(DPSolver ${SOURCES})
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(DPSolver CIMParser ${LIBS})
