set(DPSIM_VILLAS_SOURCES
	InterfaceVillas.cpp
	InterfaceWorkerVillas.cpp
	InterfaceVillasQueueless.cpp
)

add_library(dpsim-villas ${DPSIM_VILLAS_SOURCES})
target_link_libraries(dpsim-villas PUBLIC dpsim ${VILLASNODE_LIBRARIES} pthread)
target_include_directories(dpsim-villas PUBLIC ../include ${VILLASNODE_INCLUDE_DIRS})
target_compile_options(dpsim-villas PUBLIC ${DPSIM_CXX_FLAGS})
