find_package(Doxygen)

if(DOXYGEN_FOUND)
	configure_file(
		"${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in"
		"${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
		@ONLY
	)

	# HTML output directory
	set(DOXYGEN_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/doxygen")
	file(MAKE_DIRECTORY ${DOXYGEN_HTML_DIR})

	add_custom_target(docs_cxx
		COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM
	)

	install(DIRECTORY ${DOXYGEN_HTML_DIR}
		DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen
		OPTIONAL
	)
endif()
