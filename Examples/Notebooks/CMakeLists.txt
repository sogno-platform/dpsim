find_program(JUPYTER jupyter)
if(WITH_PYTHON AND JUPYTER)
	set(PYTHONPATH $<TARGET_FILE_DIR:dpsim_python>:${CMAKE_CURRENT_SOURCE_DIR}/../../Source/Python/:$ENV{PYTHONPATH})

	file(GLOB_RECURSE IPYNBS
		LIST_DIRECTORIES OFF
		RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
		"*.ipynb"
	)

	set(HTMLS "")

	foreach(IPYNB ${IPYNBS})
		if (IPYNB MATCHES .ipynb_checkpoints)
			continue()
		endif()

		get_filename_component(DIR ${IPYNB} DIRECTORY)
		get_filename_component(NAME ${IPYNB} NAME_WE)

		if (NOT DIR STREQUAL "")
			set(DIR "${DIR}/")
		endif()

		set(HTML ${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${NAME}.html)

		add_custom_command(
			COMMAND
				${CMAKE_COMMAND} -E env PYTHONPATH=${PYTHONPATH}
				jupyter nbconvert
					--to html_embed
					--output-dir ${CMAKE_CURRENT_BINARY_DIR}/${DIR}
					${CMAKE_CURRENT_SOURCE_DIR}/${IPYNB}
			OUTPUT ${HTML}
			WORKING_DIRECTORY ${DIR}
			DEPENDS
				${IPYNB}
				dpsim_python
			COMMENT "Creating HTML from IPython notebook"
		)

		list(APPEND HTMLS ${HTML})
	endforeach()

	# Convert Jupyter Notebooks to HTML
	add_custom_target(notebooks
		DEPENDS ${HTMLS}
	)

	# Run Jupyter Lab
	add_custom_target(jupyter
		COMMAND
			${CMAKE_COMMAND} -E env PYTHONPATH=${PYTHONPATH}
			jupyter lab
		DEPENDS
			dpsim_python
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT Run Jupyter Lab
	)
endif()