if(NOT WIN32)
	set(TEST_SRCS
		Timer.cpp
	)
endif()

foreach(SOURCE ${TEST_SRCS})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} dpsim)
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()
