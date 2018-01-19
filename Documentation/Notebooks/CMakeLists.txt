if(WITH_PYTHON)
	add_custom_target(jupyter
		COMMAND
			${CMAKE_COMMAND} -E env PYTHONPATH=$<TARGET_FILE_DIR:dpsim_python>:${CMAKE_CURRENT_SOURCE_DIR}/../../Source/Python/:$ENV{PYTHONPATH}
			jupyter notebook
		DEPENDS
			dpsim_python
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT Open Jupyter Notebooks
	)
endif()