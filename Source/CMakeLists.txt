cmake_minimum_required(VERSION 3.0)
project(DPsim)

# library configuration etc.
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

include_directories(${EIGEN3_INCLUDE_DIR})
add_subdirectory(CIM-XML-Parser)

# static library target for everything but main (otherwise, CMake compiles all
# object files again for each test)
file(GLOB SOURCES *.cpp Components/*.cpp Examples/*.cpp)
set(SOURCES_NOMAIN ${SOURCES})
file(GLOB MAIN_FULLPATH DPsimMain.cpp)
list(REMOVE_ITEM SOURCES_NOMAIN ${MAIN_FULLPATH})
add_library(DPsim STATIC ${SOURCES_NOMAIN})
target_link_libraries(DPsim CIMParser)

# actual executable
add_executable(DPSolver ${MAIN_FULLPATH})
target_link_libraries(DPSolver DPsim CIMParser ${LIBS})

# tests
file(GLOB TESTS Tests/*.cpp)
foreach(TEST ${TESTS})
    string(REGEX MATCH "[^/]*$" TEST_SHORT ${TEST})
    string(REPLACE ".cpp" "" TEST_BIN ${TEST_SHORT})
    message(STATUS "TEST_BIN: " ${TEST_BIN})
    add_executable(${TEST_BIN} ${TEST})
    target_link_libraries(${TEST_BIN} DPsim CIMParser ${LIBS})
endforeach()
