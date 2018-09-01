cmake_minimum_required(VERSION 3.5)
project(DPsim CXX)

# Enable C++11 support
set(CMAKE_CXX_STANDARD 17)
add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake;${PROJECT_SOURCE_DIR}/Dependencies/libcps/CMake")

set(PROJECT_AUTHOR "Institute for Automation of Complex Power Systems, RWTH Aachen University")
set(PROJECT_COPYRIGHT "2017, Institute for Automation of Complex Power Systems, RWTH Aachen University")

include(CheckCXXCompilerFlag)
if(MSVC)	
	check_cxx_compiler_flag("/W4 /WX" CXX_SUPPORTS_WERROR)

	if(CXX_SUPPORTS_WERROR)
	# TODO activate this again after fixing warnings
	#		set(DPSIM_CXX_FLAGS /W4 /WX)
	endif()
else()
	check_cxx_compiler_flag("-Wall -Werror" CXX_SUPPORTS_WERROR)

	if(CXX_SUPPORTS_WERROR)
		set(DPSIM_CXX_FLAGS -Wall -Werror)
	endif()
endif()

include(CheckSymbolExists)
check_symbol_exists(pipe unistd.h HAVE_PIPE)
check_symbol_exists(timerfd_create sys/timerfd.h HAVE_TIMERFD)

include(GetGitRevisionDescription)
git_describe(DPSIM_VERSION --tags --dirty=-dirty)

# Parse the version information into pieces.
string(REGEX REPLACE "^v([0-9]+)\\..*" "\\1" DPSIM_MAJOR_VERSION "${DPSIM_VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.([0-9]+).*" "\\1" DPSIM_MINOR_VERSION "${DPSIM_VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" DPSIM_PATCH_VERSION "${DPSIM_VERSION}")
set(DPSIM_SHORT_VERSION "${DPSIM_MAJOR_VERSION}.${DPSIM_MINOR_VERSION}.${DPSIM_PATCH_VERSION}")

if(DEFINED ENV{CI})
	set(GIT_HASH $ENV{CI_COMMIT_SHA})
	set(GIT_BRANCH $ENV{CI_COMMIT_REF_NAME})
	set(GIT_REF_SPEC $ENV{CI_COMMIT_REF_NAME})
else()
	git_rev_parse(GIT_BRANCH --abbrev-ref HEAD)
	get_git_head_revision(GIT_REFSPEC GIT_HASH)
endif()

string(SUBSTRING ${GIT_HASH} 0 7 GIT_HASH_SHORT)
string(TIMESTAMP YMD "%Y%m%d")

set(DPSIM_VERSION 0.1.4)
set(DPSIM_RELEASE ${YMD}git${GIT_HASH_SHORT})

add_subdirectory(Dependencies/libcps)

find_package(PythonInterp 3.6)
find_package(PythonLibs 3.6)
find_package(Sundials)

if(PythonInterp_FOUND AND PythonLibs_FOUND)
	set(Python_FOUND ON)
endif()

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(Linux_FOUND ON)
endif()

# Options
option(BUILD_SHARED_LIBS "Build shared library" OFF)
option(BUILD_EXAMPLES "Build C++ examples" ON)

option(WITH_SUNDIALS "Enable sundials solver suite"         ${Sundials_FOUND})
option(WITH_SHMEM    "Enable shared memory interface"       ${VILLASnode_FOUND})
option(WITH_RT	     "Enable real-time features"            ${HAVE_TIMERFD})
option(WITH_PYTHON   "Enable Python support"                ${Python_FOUND})
option(WITH_CIM      "Enable support for parsing CIM files" ${CIMpp_FOUND})

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/Include/dpsim/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/Include/dpsim/Config.h
)

set(INCLUDE_DIRS
	${CMAKE_CURRENT_BINARY_DIR}/Include
	${CMAKE_CURRENT_SOURCE_DIR}/Include
)

add_subdirectory(Source)
add_subdirectory(Documentation)

if(BUILD_EXAMPLES)
	add_subdirectory(Examples/Cxx)
endif(BUILD_EXAMPLES)

# Show feature summary
include(FeatureSummary)

add_feature_info(CIM	WITH_CIM	"Loading Common Information Model Files")
add_feature_info(Python WITH_PYTHON "Use DPsim as a Python module")
add_feature_info(Shmem  WITH_SHMEM  "Interface DPsim solvers via shared-memory interfaces")
add_feature_info(RT	 WITH_RT	 "Execute simulation in real time")
feature_summary(WHAT ALL VAR enabledFeaturesText)

message(STATUS "Feature summary for DPsim")
message(STATUS "${enabledFeaturesText}")

include(DPsimPackaging)
