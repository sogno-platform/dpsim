cmake_minimum_required(VERSION 3.11)
project(DPsim CXX)

set(PROJECT_AUTHOR "Institute for Automation of Complex Power Systems, RWTH Aachen University")
set(PROJECT_COPYRIGHT "2017-2020, Institute for Automation of Complex Power Systems, RWTH Aachen University")
set(PROJECT_DESCRIPTION "C++ Power System Simulation Library")
set(PROJECT_HOMEPAGE_URL "https://dpsim.fein-aachen.org/")
set(PROJECT_VCS_URL "https://github.com/dpsim-simulator/dpsim.git")

# Required for dpsim_python
if(POLICY CMP0076)
  cmake_policy(SET CMP0076 NEW)
endif()
# Allow setting Graphviz_ROOT for windows builds
if(WIN32)
    cmake_policy(SET CMP0074 NEW)
endif()

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

if(NOT WIN32)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(CheckCXXCompilerFlag)
if(MSVC)
	# Silence Visual Studio deprecation warnings
	add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
	add_definitions(-D_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)

	# Set exception handling for portability
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

option(WITH_EIGEN_MODULE "Build with Eigen as module" OFF)
option(WITH_SPDLOG_SUBMODULE "Build with Spdlog as module" OFF)
option(WITH_CIM_SUBMODULE "Build with CIMpp as module" OFF)
option(WITH_PYBIND_MODULE "Build with pybind11 as module" OFF)
option(WITH_PROFILING "Add `-pg` profiling flag to compiliation" OFF)
option(WITH_ASAN "Adds compiler flags to use the address sanitizer" OFF)
option(WITH_TSAN "Adds compiler flags to use the thread sanitizer" OFF)
option(CGMES_BUILD "Build with CGMES instead of CIM" OFF)

find_package(Threads REQUIRED)

if (WITH_EIGEN_MODULE OR WIN32)
	include(cmake/GetEigenModule.cmake)
else()
	find_package(Eigen3 3.0 REQUIRED)
endif()

if (WITH_SPDLOG_SUBMODULE OR WIN32)
	include(cmake/GetSpdlogModule.cmake)
else()
	find_package(spdlog REQUIRED)
	find_package(fmt REQUIRED)
endif()

if (WITH_CIM_SUBMODULE OR WIN32)
	include(cmake/GetCIMppModule.cmake)
else()
	find_package(CIMpp)
endif()

if (WITH_PROFILING)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
endif()

if (WITH_ASAN)
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract -fsanitize-address-use-after-scope")
	else()
		message(WARNING "WITH_ASAN is only supported when using GCC")
	endif()
endif()

if (WITH_TSAN)
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
	else()
		message(WARNING "WITH_TSAN is only supported when using GCC")
	endif()
endif()

find_package(Sundials)
find_package(OpenMP)
find_package(CUDA)
find_package(GSL)
find_package(Graphviz)
find_package(VILLASnode)
find_package(MAGMA)

if (WITH_PYBIND_MODULE)
	include(cmake/GetPybindModule.cmake)
else()
	find_package(pybind11 QUIET)
endif()

if (pybind11_FOUND)
	find_package (Python COMPONENTS Interpreter Development)
endif()

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(Linux_FOUND ON)
elseif("${CMAKE_SYSTEM}" MATCHES "Darwin")
	set(MacOS_FOUND ON)
endif()

option(BUILD_SHARED_LIBS 	"Build shared library"             	OFF)
option(DPSIM_BUILD_EXAMPLES    	"Build C++ examples"               	ON )
option(GET_GRID_DATA     	"Download grid data"               	ON )

option(WITH_JSON 			"Include json library" 				ON )
if(WITH_JSON)
	include(cmake/GetJsonLibrary.cmake)
endif()

# WITH_SPARSE and WITH_CUDA can be combined
option(WITH_SPARSE       	"Use sparse matrices in MNA-Solver"	ON )

include(CMakeDependentOption)
cmake_dependent_option(WITH_GSL     	"Enable GSL"                         	ON 	"GSL_FOUND"       	OFF)
cmake_dependent_option(WITH_SUNDIALS	"Enable sundials solver suite"       	ON 	"Sundials_FOUND"  	OFF)
cmake_dependent_option(WITH_SHMEM   	"Enable shared memory interface"     	ON 	"VILLASnode_FOUND"	OFF)
cmake_dependent_option(WITH_RT      	"Enable real-time features"          	ON 	"Linux_FOUND"     	OFF)
cmake_dependent_option(WITH_PYTHON  	"Enable Python support"              	ON 	"Python_FOUND"    	OFF)
cmake_dependent_option(WITH_CIM     	"Enable support for parsing CIM"     	ON 	"CIMpp_FOUND"     	OFF)
cmake_dependent_option(WITH_OPENMP  	"Enable OpenMP-based parallelisation"	ON 	"OPENMP_FOUND"    	OFF)
cmake_dependent_option(WITH_CUDA    	"Enable CUDA-based parallelisation"  	OFF	"CUDA_FOUND"      	OFF)
cmake_dependent_option(WITH_GRAPHVIZ	"Enable Graphviz Graphs"             	ON 	"GRAPHVIZ_FOUND"  	OFF)
cmake_dependent_option(WITH_PYBIND     	"Enable PYBIND support"            	ON 	"pybind11_FOUND"   	OFF)
cmake_dependent_option(WITH_MAGMA     	"Enable MAGMA features"            	ON 	"MAGMA_FOUND"   	OFF)

if(WITH_CUDA)
    # BEGIN OF WORKAROUND - enable cuda dynamic linking.
    # Starting with Cmake 3.17 we can use
    # set(CMAKE_CUDA_RUNTIME_LIBRARY_DEFAULT "SHARED") instead
    set( CMAKE_CUDA_FLAGS "" CACHE STRING "" )
    if ( CMAKE_CUDA_FLAGS )
            list(REMOVE_ITEM CMAKE_CUDA_FLAGS "-cudart static")
    endif()
    string(APPEND CMAKE_CUDA_FLAGS "-cudart shared")
    # END OF WORKAROUND

	enable_language(CUDA)

    # BEGIN OF WORKAROUND - enable cuda dynamic linking.
    if ( CMAKE_CUDA_HOST_IMPLICIT_LINK_LIBRARIES )
        list( REMOVE_ITEM CMAKE_CUDA_HOST_IMPLICIT_LINK_LIBRARIES "cudart_static" )
        list( REMOVE_ITEM CMAKE_CUDA_HOST_IMPLICIT_LINK_LIBRARIES "cudadevrt" )
        list( APPEND CMAKE_CUDA_HOST_IMPLICIT_LINK_LIBRARIES "cudart" )
    endif()
    if ( CMAKE_CUDA_IMPLICIT_LINK_LIBRARIES )
        list( REMOVE_ITEM CMAKE_CUDA_IMPLICIT_LINK_LIBRARIES "cudart_static" )
        list( REMOVE_ITEM CMAKE_CUDA_IMPLICIT_LINK_LIBRARIES "cudadevrt" )
        list( APPEND CMAKE_CUDA_IMPLICIT_LINK_LIBRARIES "cudart" )
    endif()
    if ( CUDA_LIBRARIES )
        list ( REMOVE_ITEM CUDA_LIBRARIES "${CUDA_cudart_static_LIBRARY}")
        list ( APPEND CUDA_LIBRARIES "${CUDA_CUDART_LIBRARY}")
    endif()
    # END OF WORKAROUND
endif()

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/Include/dpsim/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/Include/dpsim/Config.h
)

if(WITH_SHMEM)
	include(cmake/GetVillasDPsim.cmake)
endif()

set(DPSIM_INCLUDE_DIRS
	${CMAKE_CURRENT_BINARY_DIR}/Include
	${CMAKE_CURRENT_SOURCE_DIR}/Include
)

add_subdirectory(models)
add_subdirectory(Source)
add_subdirectory(docs)

if(DPSIM_BUILD_EXAMPLES)
	add_subdirectory(Examples)
endif(DPSIM_BUILD_EXAMPLES)

if(WITH_PYBIND)
	set(PYBIND11_CPP_STANDARD -std=c++11)

	pybind11_add_module(dpsimpy Source/pybind/main.cpp)	
	target_link_libraries(dpsimpy PRIVATE dpsim)

	if(WITH_SHMEM)
		pybind11_add_module(dpsimpyvillas ${CMAKE_BINARY_DIR}/_deps/dpsim-villas-src/src/pybind-dpsim-villas.cpp)
		target_link_libraries(dpsimpyvillas PRIVATE dpsim)
	endif()	
endif()

include(cmake/GetGridData.cmake)

message(VERBOSE "Compiler Release Flags: " 	${CMAKE_CXX_FLAGS_RELEASE})
message(VERBOSE "Compiler Debug Flags: " 	${CMAKE_CXX_FLAGS_DEBUG})

if(TOPLEVEL_PROJECT)
	include(FeatureSummary)
	add_feature_info(CIM		WITH_CIM			"Loading Common Information Model Files")
	add_feature_info(Python 	WITH_PYTHON 		"Use DPsim as a Python module")
	add_feature_info(Shmem  	WITH_SHMEM  		"Interface DPsim solvers via shared-memory interfaces")
	add_feature_info(RT	    	WITH_RT     		"Extended real-time features")
	add_feature_info(JSON		WITH_JSON  			"Use JSON library")
	add_feature_info(GSL		WITH_GSL  			"Use GNU Scientific library")
	add_feature_info(Graphviz  	WITH_GRAPHVIZ  		"Graphviz Graphs")
	add_feature_info(Sundials  	WITH_SUNDIALS  		"Sundials solvers")
	add_feature_info(PYBIND 	WITH_PYBIND 		"Use DPsim as a PYBIND module")
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
