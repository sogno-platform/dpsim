cmake_minimum_required(VERSION 3.6)
project(CPowerSystems CXX)

if(${CMAKE_VERSION} VERSION_GREATER "3.12.0")
  # Allow setting Graphviz_ROOT for windows builds
  cmake_policy(SET CMP0074 NEW)
endif()

# Enable C++11 support
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

include(CheckCXXCompilerFlag)
if(MSVC)
	check_cxx_compiler_flag("/W4 /WX" CXX_SUPPORTS_WERROR)

	if(CXX_SUPPORTS_WERROR)
	# TODO activate this again after fixing warnings
	#	set(CPS_CXX_FLAGS /W4 /WX)
	endif()
else()
	check_cxx_compiler_flag("-Wall -Werror" CXX_SUPPORTS_WERROR)

	if(CXX_SUPPORTS_WERROR)
		set(CPS_CXX_FLAGS -Wall -Werror)
	endif()
endif()

set(Python_ADDITIONAL_VERSIONS 3.5 3.6 3.7)

find_package(Threads REQUIRED)
find_package(PythonInterp 3.5)
find_package(PythonLibs 3.5)
find_package(Graphviz)
find_package(Sundials)
find_package(GSL)
find_package(NumPy)

if(PythonInterp_FOUND AND PythonLibs_FOUND)
	set(Python_FOUND ON)
endif()

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(Linux_FOUND ON)
elseif("${CMAKE_SYSTEM}" MATCHES "Darwin")
	set(MacOS_FOUND ON)
endif()

include(CMakeDependentOption)
cmake_dependent_option(WITH_NUMPY    	"Enable NumPy"							ON "NumPy_FOUND"		OFF)
cmake_dependent_option(WITH_GSL			"Enable GSL"        					ON "GSL_FOUND"			OFF)
cmake_dependent_option(WITH_SUNDIALS	"Enable sundials solver suite"			ON "Sundials_FOUND"		OFF)
cmake_dependent_option(WITH_PYTHON		"Enable Python support"					ON "Python_FOUND"		OFF)
cmake_dependent_option(WITH_CIM			"Enable support for parsing CIM files"	ON "CIMpp_FOUND"		OFF)
cmake_dependent_option(WITH_GRAPHVIZ	"Enable support for Graphviz Graphs"	ON "Graphviz_FOUND"		OFF)

# Show feature summary
include(FeatureSummary)

add_feature_info(CIM       WITH_CIM       "Loading Common Information Model Files")
add_feature_info(Graphviz  WITH_GRAPHVIZ  "Graphviz Graphs")
add_feature_info(Python    WITH_PYTHON    "Use DPsim as a Python module")
add_feature_info(Sundials  WITH_SUNDIALS  "Sundials solvers")

if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
	feature_summary(WHAT ALL VAR enabledFeaturesText)
	message(STATUS "Feature summary for CPS")
	message(STATUS "${enabledFeaturesText}")
endif()

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/Include/cps/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/Include/cps/Config.h
)

set(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/Include
	${CMAKE_CURRENT_BINARY_DIR}/Include
	${EIGEN3_INCLUDE_DIR}
	${SPDLOG_INCLUDE_DIR}
)

set(LIBRARIES
	Threads::Threads
)

if(Linux_FOUND OR MacOS_FOUND)
	# TODO This doesn't actually depend on which OS or compiler is used, but which
	# stdlib. For now, under Linux only GNU's libstdc++ seems to work anyway.
	# With LLVM's libc++, -lc++fs should probably be used
	list(APPEND LIBRARIES stdc++fs)
endif()

add_subdirectory(Source)
add_subdirectory(Examples)
add_subdirectory(Tests)
