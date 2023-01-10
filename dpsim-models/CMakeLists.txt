configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/include/dpsim-models/Config.h.in
	${CMAKE_CURRENT_BINARY_DIR}/include/dpsim-models/Config.h
)

set(MODELS_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}/include
)

set(MODELS_LIBRARIES
	filesystem
	Threads::Threads
	Eigen3::Eigen
	spdlog::spdlog
)

add_subdirectory(src)
