if(WITH_GRAPHVIZ AND WITH_CIM)
	add_executable(cimviz cimviz.cpp)
	target_link_libraries(cimviz PUBLIC dpsim-models CIMPP::cimpp ${GRAPHVIZ_LIBRARIES})
endif()
