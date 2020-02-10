cmake_minimum_required(VERSION 3.5)
project(DPsim CXX)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
	set(TOPLEVEL_PROJECT ON)
else()
		set(TOPLEVEL_PROJECT OFF)
endif()

# Enable C++11 support
set(CMAKE_CXX_STANDARD 11)
if(CMAKE_CXX_COMPILER_ID STREQUAL Intel)
	# needed by icc, but for some reason not added by CMake
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
add_definitions(-D_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake;${PROJECT_SOURCE_DIR}/Dependencies/libcps/CMake")

set(PROJECT_AUTHOR "Institute for Automation of Complex Power Systems, RWTH Aachen University")
set(PROJECT_COPYRIGHT "2017-2020, Institute for Automation of Complex Power Systems, RWTH Aachen University")
set(PROJECT_DESCRIPTION "C++ Power System Simulation Library")
set(PROJECT_HOMEPAGE_URL "http://www.fein-aachen.org/projects/dpsim/")
set(PROJECT_VCS_URL "https://git.rwth-aachen.org/acs/public/simulation/dpsim")

include(CheckCXXCompilerFlag)
if(MSVC)	
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")

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
check_symbol_exists(getopt_long getopt.h HAVE_GETOPT)

# Get version info and buildid from Git
include(GetVersion)
GetVersion(${PROJECT_SOURCE_DIR} "DPSIM")

find_package(Eigen3 3.0 REQUIRED)
option(SPDLOG_BUILD_TESTING "Build spdlog tests" OFF)
option(SPDLOG_BUILD_BENCH "Build spdlog benchmarks" OFF)
option(SPDLOG_BUILD_EXAMPLES "Build spdlog examples" OFF)
# Workaround for spdlog in Windows
# A custom FindSpdlog does not work currently
option(WITH_SPDLOG_SUBMODULE "Build with Spdlog as submodule" OFF)
if (WITH_SPDLOG_SUBMODULE OR WIN32)	
	set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/spdlog/include)
else()
	find_package(spdlog REQUIRED)
endif()
find_package(CIMpp)
add_subdirectory(Dependencies/libcps)

find_package(PythonInterp 3.6)
find_package(PythonLibs 3.6)
find_package(Sundials)

find_package(OpenMP)

find_package(GSL)
find_package(VILLASnode)

if(PythonInterp_FOUND AND PythonLibs_FOUND)
	set(Python_FOUND ON)
endif()

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(Linux_FOUND ON)
endif()

# Options
include(CMakeDependentOption)

option(BUILD_SHARED_LIBS "Build shared library" OFF)
option(BUILD_EXAMPLES "Build C++ examples" ON)
option(COMPARE_REFERENCE "Download reference results and compare" OFF)

cmake_dependent_option(WITH_GSL			"Enable GSL"        					ON "GSL_FOUND"			OFF)
cmake_dependent_option(WITH_SUNDIALS	"Enable sundials solver suite"			ON "Sundials_FOUND"		OFF)
cmake_dependent_option(WITH_SHMEM		"Enable shared memory interface"		ON "VILLASnode_FOUND"	OFF)
cmake_dependent_option(WITH_RT			"Enable real-time features"				ON "Linux_FOUND"		OFF)
cmake_dependent_option(WITH_PYTHON		"Enable Python support"					ON "Python_FOUND"		OFF)
cmake_dependent_option(WITH_CIM			"Enable support for parsing CIM files"	ON "CIMpp_FOUND"		OFF)
cmake_dependent_option(WITH_OPENMP		"Enable OpenMP-based parallelisation"	ON "OPENMP_FOUND"		OFF)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/Include/dpsim/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/Include/dpsim/Config.h
)

set(INCLUDE_DIRS
	${CMAKE_CURRENT_BINARY_DIR}/Include
	${CMAKE_CURRENT_SOURCE_DIR}/Include
)

add_subdirectory(Tests)
add_subdirectory(Source)
add_subdirectory(Documentation)

if(BUILD_EXAMPLES)
	add_subdirectory(Examples)
endif(BUILD_EXAMPLES)

if(COMPARE_REFERENCE)
	include(ExternalProject)
	ExternalProject_Add(reference-results
	  GIT_REPOSITORY    https://git.rwth-aachen.de/acs/public/simulation/reference-results.git
	  #GIT_TAG           origin/master
	  PREFIX			reference-results
	  CONFIGURE_COMMAND	""
	  BUILD_COMMAND		""
	  INSTALL_COMMAND	""
	)
endif()

message("Compiler Release Flags:")
message(${CMAKE_CXX_FLAGS_RELEASE})
message("Compiler Debug Flags:")
message(${CMAKE_CXX_FLAGS_DEBUG})

if(TOPLEVEL_PROJECT)
	# Show feature summary
	include(FeatureSummary)

	add_feature_info(CIM	WITH_CIM	"Loading Common Information Model Files")
	add_feature_info(Python WITH_PYTHON "Use DPsim as a Python module")
	add_feature_info(Shmem  WITH_SHMEM  "Interface DPsim solvers via shared-memory interfaces")
	add_feature_info(RT	    WITH_RT     "Extended real-time features")
	add_feature_info(GSL	WITH_GSL     "Use GNU Scientific library")
	feature_summary(WHAT ALL VAR enabledFeaturesText)

	if (FOUND_GIT_VERSION)
		message(STATUS "Building ${CMAKE_PROJECT_NAME}:")
		message(STATUS "Feature summary for DPsim")
		message(STATUS "${enabledFeaturesText}")
		message(STATUS "  VERSION:         ${DPSIM_VERSION}")
		message(STATUS "  RELEASE:         ${DPSIM_RELEASE}")
		message(STATUS "  GIT_REV:         ${DPSIM_GIT_REV}")
		message(STATUS "  GIT_BRANCH:      ${DPSIM_GIT_BRANCH}")
		message(STATUS "  VARIANT:         ${DPSIM_VARIANT}")
		message(STATUS "  BUILD_ID:        ${DPSIM_BUILD_ID}")
		message(STATUS "  BUILD_DATE:      ${DPSIM_BUILD_DATE}")
	endif()
endif()

include(DPsimPackaging)
