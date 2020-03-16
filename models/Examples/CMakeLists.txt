set(LIBRARIES cps)

# targets
if(WITH_CIM)
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${CIMPP_LIBRARIES})
endif()

if(WITH_GRAPHVIZ)
	list(APPEND INCLUDE_DIRS ${GRAPHVIZ_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${GRAPHVIZ_LIBRARIES})
endif()

if(WITH_GRAPHVIZ AND WITH_CIM)
	add_executable(cimviz cimviz.cpp)

	target_link_libraries(cimviz PUBLIC ${LIBRARIES})
	target_include_directories(cimviz PUBLIC ${INCLUDE_DIRS})
	
	target_compile_options(cimviz PUBLIC ${CPS_CXX_FLAGS})
endif()
