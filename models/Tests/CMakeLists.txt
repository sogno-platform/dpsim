option(WITH_TESTS "Build Tests" ON)

if(WITH_TESTS)
	set(TEST_SRCS
		FIRFilter.cpp
		Exciter.cpp
		TurbineGovernor.cpp
		DP_Ph1_Inverter_Test.cpp
	)
endif()

foreach(SOURCE ${TEST_SRCS})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} cps)
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${CPS_CXX_FLAGS})
endforeach()
