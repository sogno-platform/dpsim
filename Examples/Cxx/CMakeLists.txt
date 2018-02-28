set(LIBRARIES "dpsim")

set(INCLUDE_DIRS
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

# targets
set(CIRCUIT_SOURCES
	Circuits/DP_IdealVS_PILine1.cpp
	Circuits/DP_IdealVS_R1.cpp
	Circuits/DP_IdealVS_R2.cpp
	Circuits/DP_IdealVS_RLC1.cpp
	Circuits/DP_IdealVS_RXLine1.cpp
	Circuits/DP_IdealVS_RXLine2.cpp
	Circuits/DP_ResVS_PILine1.cpp
	Circuits/DP_ResVS_RL1.cpp
	Circuits/DP_ResVS_RL2.cpp
	Circuits/DP_ResVS_RLC1.cpp
	Circuits/DP_ResVS_RXLine1.cpp
	Circuits/EMT_IdealVS_R1.cpp
	Circuits/EMT_ResVS_RXLine_Switch1.cpp
)

set(SYNCGEN_SOURCES
	SynchronGenerator/DP_Multimachine.cpp
	SynchronGenerator/DP_SynchronGenerator_BalancedResLoad.cpp
	SynchronGenerator/DP_SynchronGenerator_Dq_ThreePhaseFault.cpp
	SynchronGenerator/DP_SynchronGenerator_ExciterAndTurbine.cpp
	SynchronGenerator/DP_SynchronGenerator_Simplified_ThreePhaseFault.cpp
	SynchronGenerator/DP_SynchronGenerator_SimpThreePhaseFault.cpp
	SynchronGenerator/DP_SynchronGenerator_VBR.cpp
	SynchronGenerator/DP_SynchronGenerator_VBR_ThreePhaseFault.cpp
	SynchronGenerator/DP_SynchronousGenerator_Dq_LoadChange.cpp
	SynchronGenerator/DP_SynchronousGenerator_VBR_LoadChange
	SynchronGenerator/EMT_Multimachine.cpp
	SynchronGenerator/EMT_Multimachine_Classical.cpp
	SynchronGenerator/EMT_SynchronGenerator_BalancedResLoad.cpp
	SynchronGenerator/EMT_SynchronGenerator_Dq_ThreePhaseFault
	SynchronGenerator/EMT_SynchronGenerator_ExciterAndTurbine.cpp
	SynchronGenerator/EMT_SynchronGenerator_PhaseToPhaseFault.cpp
	SynchronGenerator/EMT_SynchronGenerator_Simplified_ThreePhaseFault.cpp
	SynchronGenerator/EMT_SynchronGenerator_VBR_ThreePhaseFault.cpp
	SynchronGenerator/EMT_SynchronousGenerator_Dq_LoadChange.cpp
	SynchronGenerator/EMT_SynchronousGenerator_VBR_LoadChange.cpp
	SynchronGenerator/ExciterAndTurbine.cpp
	
	


)

set(VARFREQ_SOURCES
	VariableTimeStep/RXLine_LoadStep_FreqStep_1.cpp
	VariableTimeStep/RXLine_LoadStep_FreqStep_2.cpp
)

if(WITH_RT)
	list(APPEND LIBRARIES "-lrt")

	set(RT_SOURCES
		RealTime/RT_DP_ResVS_RL1.cpp
	)
endif()

if(WITH_SHMEM)
	list(APPEND LIBRARIES ${VILLASNODE_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})

	set(SHMEM_SOURCES
		Shmem/ShmemExample.cpp
		Shmem/ShmemDistributedReference.cpp
		Shmem/ShmemDistributedVillas.cpp
		Shmem/ShmemDistributedDirect.cpp
		Shmem/ShmemRealTime.cpp
	)
endif()

if(WITH_CIM)
	list(APPEND LIBRARIES ${CIMPP_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})

	set(CIM_SOURCES
		CIM/CIM.cpp
	)
endif()

foreach(SOURCE ${CIRCUIT_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES} ${SHMEM_SOURCES} ${RT_SOURCES} ${CIM_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
endforeach()

