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

file(GLOB_RECURSE HEADER_FILES include/*.h)

target_sources(dpsim-models PUBLIC
	FILE_SET public_headers
	TYPE HEADERS
	BASE_DIRS include
	FILES "${HEADER_FILES}"
)

install(TARGETS dpsim-models
	EXPORT dpsim-models
	FILE_SET public_headers DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
