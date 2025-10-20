set(DPSIM_VILLAS_SOURCES
	InterfaceVillas.cpp
	InterfaceWorkerVillas.cpp
	InterfaceVillasQueueless.cpp
)

add_library(dpsim-villas ${DPSIM_VILLAS_SOURCES})

if(NOT VILLASNODE_LIBRARIES)
	message(FATAL_ERROR "villas-node is required when building DPsim with VILLAS support.")
endif()

if(NOT VILLASCOMMON_LIBRARIES)
	message(FATAL_ERROR "villas-common is required when building DPsim with VILLAS support.")
endif()

set(DPSIM_VILLAS_LIBRARIES
	dpsim
	${VILLASNODE_LIBRARIES}
	${VILLASCOMMON_LIBRARIES}
	pthread
)
if(VILLASFPGA_LIBRARIES)
	list(APPEND DPSIM_VILLAS_LIBRARIES ${VILLASFPGA_LIBRARIES})
endif()

target_link_libraries(dpsim-villas PUBLIC ${DPSIM_VILLAS_LIBRARIES})
target_include_directories(dpsim-villas PUBLIC ../include ${VILLASNODE_INCLUDE_DIRS})
target_compile_options(dpsim-villas PUBLIC ${DPSIM_CXX_FLAGS})
