configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/include/dpsim-models/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/include/dpsim-models/Config.h
)

set(MODELS_INCLUDE_DIRS
	${EIGEN3_INCLUDE_DIR}
	${KLU_INCLUDE_DIR}
	${SPDLOG_INCLUDE_DIR}
	${KLU_INCLUDE_DIR}
)

set(MODELS_LIBRARIES
	Threads::Threads
)

if (WITH_SPDLOG_SUBMODULE OR WIN32)
	list(APPEND MODELS_LIBRARIES spdlog::spdlog)
else()
	list(APPEND MODELS_LIBRARIES spdlog::spdlog fmt::fmt)
endif()

if (WITH_KLU)
	list(APPEND MODELS_LIBRARIES klu)
endif()

if (WITH_EIGEN_SUBMODULE OR WIN32)
	# do not add eigen here
else()
	list(APPEND MODELS_LIBRARIES Eigen3::Eigen)
endif()

if(WITH_KLU)
	list(APPEND CPS_LIBRARIES klu)
endif()

if(Linux_FOUND OR MacOS_FOUND)
	# TODO This doesn't actually depend on which OS or compiler is used, but which
	# stdlib. For now, under Linux only GNU's libstdc++ seems to work anyway.
	# With LLVM's libc++, -lc++fs should probably be used
	list(APPEND MODELS_LIBRARIES stdc++fs)
endif()

add_subdirectory(src)
