set(SIGNALS_SRCS
	FIRFilter.cpp
	Exciter.cpp
	TurbineGovernor.cpp
)

foreach(SOURCE ${SIGNALS_SRCS})
	get_filename_component(TARGET ${SOURCE} NAME_WE)
	add_executable(${TARGET} ${SOURCE})
	target_link_libraries(${TARGET} cps)
endforeach()
