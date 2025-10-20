set(LIBRARIES "dpsim-villas")

if(NOT WIN32)
	# Needed so CIMParser and Arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

if(NOT VILLASNODE_LIBRARIES)
	message(FATAL_ERROR "villas-node is required when building DPsim VILLAS examples.")
endif()

if(NOT VILLASCOMMON_LIBRARIES)
	message(FATAL_ERROR "villas-common is required when building DPsim VILLAS examples.")
endif()

list(APPEND LIBRARIES
	${VILLASNODE_LIBRARIES}
	${VILLASCOMMON_LIBRARIES}
	"jansson"
	"uuid"
)

if(VILLASFPGA_LIBRARIES)
	list(APPEND LIBRARIES ${VILLASFPGA_LIBRARIES})
	list(APPEND LIBRARIES "xil")
endif()
list(APPEND INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})


set(DPSIM_VILLAS_EXAMPLE_SOURCES
	FileExample.cpp
	MqttExample.cpp
	SharedMemExample.cpp
)

set(DPSIM_VILLAS_DEPRECATED_BASE
	deprecated/ShmemExample.cpp
	deprecated/ShmemDistributedReference.cpp
	deprecated/ShmemDistributedDirect.cpp
	deprecated/ShmemDistributedVillas.cpp
	deprecated/ShmemControllableSource.cpp
	deprecated/ShmemControllableFiltSource.cpp
)

set(DPSIM_VILLAS_DEPRECATED_CIM
	deprecated/Shmem_CIGRE_MV_PowerFlowTest.cpp
	deprecated/Shmem_CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
	deprecated/Shmem_WSCC-9bus.cpp
	deprecated/Shmem_WSCC-9bus_Ctrl.cpp
	deprecated/Shmem_WSCC-9bus_CtrlDist.cpp
)

list(APPEND DPSIM_VILLAS_EXAMPLE_SOURCES ${DPSIM_VILLAS_DEPRECATED_BASE})

if(WITH_CIM)
	list(APPEND DPSIM_VILLAS_EXAMPLE_SOURCES ${DPSIM_VILLAS_DEPRECATED_CIM})
else()
	message(STATUS "Skipping deprecated CIM-based shmem examples because WITH_CIM is disabled")
endif()

if(VILLASFPGA_LIBRARIES)
	list(APPEND DPSIM_VILLAS_EXAMPLE_SOURCES
		FpgaExample.cpp
		FpgaCosim3PhInfiniteBus.cpp
		FpgaCosimulation.cpp
	)

	if(WITH_CIM)
		list(APPEND DPSIM_VILLAS_EXAMPLE_SOURCES Fpga9BusHil.cpp)
	endif()
elseif(WITH_CIM)
	message(STATUS "Skipping FPGA-related CIM example because villas-fpga was not found.")
endif()

if(WITH_CIM)
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})
endif()

foreach(SOURCE ${DPSIM_VILLAS_EXAMPLE_SOURCES} ${CIM_DPSIM_VILLAS_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()
