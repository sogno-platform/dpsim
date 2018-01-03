include_directories(
	${EIGEN3_INCLUDE_DIR}
	"${CMAKE_CURRENT_BINARY_DIR}/../../Source"
	"${CMAKE_CURRENT_SOURCE_DIR}/../../Source"
)

if (NOT WIN32)
	# needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "-fPIC")
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif()

# enable C++11 support
set(CMAKE_CXX_STANDARD 11)

# targets
set(REFERENCE_SOURCES
	Circuits/DP_IdealVS_PILine1.cpp
	Circuits/DP_IdealVS_R1.cpp
	Circuits/DP_IdealVS_RLC1.cpp
	Circuits/DP_IdealVS_RXLine1.cpp
	Circuits/DP_IdealVS_RXLine2.cpp
	Circuits/DP_ResVS_PILine1.cpp
	Circuits/DP_ResVS_RL1.cpp
	Circuits/DP_ResVS_RLC1.cpp
	Circuits/DP_ResVS_RXLine1.cpp
	Circuits/EMT_IdealVS_R1.cpp
	Circuits/EMT_ResVS_RXLine_Switch1.cpp
)

set(SYNCGEN_SOURCES
	SynchronGenerator/SimpSynGenThreePhaseFault.cpp
	SynchronGenerator/SynGenDPBalancedResLoad.cpp
	SynchronGenerator/SynGenDPThreePhaseFault.cpp
	SynchronGenerator/SynGenBalancedResLoad.cpp
	SynchronGenerator/SynGenPhaseToPhaseFault.cpp
	SynchronGenerator/SynGenThreePhaseFault.cpp
	SynchronGenerator/SynGenVBR.cpp
	SynchronGenerator/SynGenVBRDP.cpp
)

set(VARFREQ_SOURCES
	VariableTimeStep/RXLine_LoadStep_FreqStep_1.cpp
	VariableTimeStep/RXLine_LoadStep_FreqStep_2.cpp
)

set(RT_SOURCES
	Circuits/RT_DP_ResVS_RL1.cpp
)

set(SHMEM_SOURCES
	Shmem/ShmemDistributed.cpp
	Shmem/ShmemDistributedDirect.cpp
	Shmem/ShmemDistributedRef.cpp
	Shmem/ShmemExample.cpp
	Shmem/ShmemMain.cpp
	Shmem/ShmemRTExample.cpp
)

foreach(SOURCE ${REFERENCE_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} dpsim)
endforeach()

# CIMpp
find_package(CIMpp)
if(CIMpp_FOUND)
	add_executable(CIM CIM.cpp)

	target_include_directories(CIM PUBLIC ${CIMPP_INCLUDE_DIRS})
	target_link_libraries(CIM ${CIMPP_LIBRARIES})
	target_link_libraries(CIM dpsim)
endif()

# Linux for shared memory and real-time
if(LINUX)
	foreach(SOURCE ${SHMEM_SOURCES} ${RT_SOURCES})
		get_filename_component(TARGET ${SOURCE} NAME_WE)

		add_executable(${TARGET} ${SOURCE})

		target_link_libraries(${TARGET} SHARED dpsim)
	endforeach()
endif()
