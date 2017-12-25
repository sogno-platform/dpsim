cmake_minimum_required(VERSION 3.5)
project(DPsim CXX)

# Enable C++11 support
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_AUTHOR "Institute for Automation of Complex Power Systems, RWTH Aachen Univerity")
set(PROJECT_COPYRIGHT "2017, Institute for Automation of Complex Power Systems, RWTH Aachen Univerity")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

include(GetGitRevisionDescription)
git_describe(DPSIM_VERSION --tags --dirty=-dirty)
git_rev_parse(GIT_BRANCH --abbrev-ref HEAD)
get_git_head_revision(GIT_REFSPEC GIT_HASH)

string(SUBSTRING ${GIT_HASH} 0 7 GIT_HASH_SHORT)
string(TIMESTAMP YMD "%Y%m%d")

set(DPSIM_RELEASE ${YMD}git${GIT_HASH_SHORT})

# {arse the version information into pieces.
string(REGEX REPLACE "^v([0-9]+)\\..*" "\\1" DPSIM_MAJOR_VERSION "${DPSIM_VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.([0-9]+).*" "\\1" DPSIM_MINOR_VERSION "${DPSIM_VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" DPSIM_PATCH_VERSION "${DPSIM_VERSION}")
set(DPSIM_SHORT_VERSION "${DPSIM_MAJOR_VERSION}.${DPSIM_MINOR_VERSION}.${DPSIM_PATCH_VERSION}")

set(Python_ADDITIONAL_VERSIONS 3.5 3.6 3.7)
find_package(PythonInterp 3.5)
find_package(PythonLibs 3.5)

set(CIM_DIR 16v29a)
find_package(CIMpp)

find_package(Eigen3 3.0 REQUIRED)
find_package(VILLASnode)

if(PythonInterp_FOUND AND PythonLibs_FOUND)
	set(Python_FOUND ON)
endif()

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(Linux_FOUND ON)
endif()

# Options
option(BUILD_SHARED_LIBS "Build shared library" OFF)

option(WITH_RT     "Enable real-time features"            ${Linux_FOUND})
option(WITH_SHMEM  "Enable shared memory interface"       ${VILLASnode_FOUND})
option(WITH_PYTHON "Enable Python support"                ${Python_FOUND})
option(WITH_CIM    "Enable support for parsing CIM files" ${CIMpp_FOUND})

add_subdirectory(Documentation)
add_subdirectory(Source)
add_subdirectory(Examples/Cxx)

# Show feature summary
include(FeatureSummary)

add_feature_info(CIM    WITH_CIM    "Loading Common Information Model Files")
add_feature_info(Python WITH_PYTHON "Use DPsim as a Python module")
add_feature_info(Shmem  WITH_SHMEM  "Interface DPsim solvers via shared-memory interfaces")
add_feature_info(RT     WITH_RT     "Execute simulation in real time")
feature_summary(WHAT ALL VAR enabledFeaturesText)

message(STATUS "${enabledFeaturesText}")

# CPack configuration
include(DPsimPackaging)
