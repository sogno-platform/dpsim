if(WITH_PYBIND)
	find_package(Sphinx)

	if(SPHINX_EXECUTABLE)
		# configured documentation tools and intermediate build results
		set(BINARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_build")

		# Sphinx cache with pickled ReST documents
		set(SPHINX_CACHE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_doctrees")

		# HTML output directory
		set(SPHINX_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/html")

		set(SPHINX_TITLE "${PROJECT_NAME} docs")

		configure_file(
			"${CMAKE_CURRENT_SOURCE_DIR}/conf.py.in"
			"${BINARY_BUILD_DIR}/conf.py"
			@ONLY
		)

		# We add the path of the freshly build dpsimpy extension to Python path here
		add_custom_target(docs
			COMMAND
				${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_BINARY_DIR}:$ENV{PYTHONPATH}
				${SPHINX_EXECUTABLE}
					-q -b html
					-c "${BINARY_BUILD_DIR}"
					-d "${SPHINX_CACHE_DIR}"
					"${CMAKE_CURRENT_SOURCE_DIR}"
					"${SPHINX_HTML_DIR}"
			COMMENT "Building HTML documentation with Sphinx"
			DEPENDS
				dpsimpy
		)

		install(DIRECTORY ${SPHINX_HTML_DIR}
			DESTINATION ${CMAKE_INSTALL_DOCDIR}/sphinx
			OPTIONAL
		)
	endif()
endif()
