cmake_minimum_required(VERSION 3.0)
project(DPsim)

find_package(Eigen3 REQUIRED)
add_subdirectory(CIM-XML-Parser)

file(GLOB SOURCES *.cpp Components/*.cpp Examples/*.cpp)

add_executable(DPSolver ${SOURCES})
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(DPSolver CIMParser -lrt -lvillas-ext)
