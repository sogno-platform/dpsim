cmake_minimum_required(VERSION 3.5)
project(DPsim CXX)

# enable C++11 support
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_AUTHOR "Institute for Automation of Complex Power Systems, RWTH Aachen Univerity")
set(PROJECT_COPYRIGHT "2017, Institute for Automation of Complex Power Systems, RWTH Aachen Univerity")

set(DPSIM_MAJOR_VERSION 0)
set(DPSIM_MINOR_VERSION 1)
set(DPSIM_PATCH_VERSION 0)
set(DPSIM_VERSION ${DPSIM_MAJOR_VERSION}.${DPSIM_MINOR_VERSION}.${DPSIM_PATCH_VERSION})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

# library configuration etc.
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Libraries/eigen)
    set(EIGEN3_INCLUDE_DIR Libraries/eigen)
else()
    find_package(Eigen3 REQUIRED)
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(Documentation)
add_subdirectory(Source)
add_subdirectory(Examples/C++)

# CPack configuration
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "My funky project")
SET(CPACK_PACKAGE_VENDOR ${PROJECT_AUTHOR})
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR "3")
SET(CPACK_PACKAGE_VERSION_PATCH "2")


# Show feature summary
include(FeatureSummary)
feature_summary(WHAT ALL
    VAR enabledFeaturesText)
message(STATUS "${enabledFeaturesText}")