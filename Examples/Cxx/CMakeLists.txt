set(LIBRARIES "dpsim")

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
	Circuits/DP_IdealVS_R_1.cpp
	Circuits/DP_CS_R_1.cpp
	Circuits/DP_IdealVS_RXLine_1.cpp
	Circuits/DP_IdealVS_Trafo_1.cpp
	Circuits/DP_ResVS_RL_Switch_1.cpp
	Circuits/EMT_IdealVS_R_1.cpp
	Circuits/EMT_ResVS_RL_Switch_1.cpp
)

set(SYNCGEN_SOURCES
	SynchronGenerator/DP_SynchronGenerator_BalancedResLoad.cpp
	SynchronGenerator/DP_SynchronGenerator_ExciterAndTurbine.cpp
	SynchronGenerator/DP_SynchronGenerator_SimpThreePhaseFault.cpp
	SynchronGenerator/DP_SynchronGenerator_ThreePhaseFault.cpp
	SynchronGenerator/DP_SynchronGenerator_VBR.cpp
	SynchronGenerator/DP_SynchronGenerator_Simplified_ThreePhaseFault.cpp
	SynchronGenerator/EMT_SynchronGenerator_BalancedResLoad.cpp
	SynchronGenerator/EMT_SynchronGenerator_ExciterAndTurbine.cpp
	SynchronGenerator/EMT_SynchronGenerator_PhaseToPhaseFault.cpp
	SynchronGenerator/EMT_SynchronGenerator_ThreePhaseFault.cpp
	SynchronGenerator/EMT_SynchronGenerator_VBR_ThreePhaseFault.cpp
	SynchronGenerator/EMT_SynchronGenerator_Simplified_ThreePhaseFault.cpp
	SynchronGenerator/EMT_Multimachine.cpp
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
		Shmem/ShmemDistributedDirect.cpp
	)

	if (WITH_RT)
		list(APPEND SHMEM_SOURCES
			Shmem/ShmemDistributedVillas.cpp
			Shmem/ShmemControllableSource.cpp
			Shmem/ShmemControllableFiltSource.cpp
			Shmem/ShmemRealTime.cpp
		)
	endif()
endif()

if(WITH_CIM)
	list(APPEND LIBRARIES ${CIMPP_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})

	set(CIM_SOURCES
		CIM/cmd_CIM_input.cpp
		CIM/WSCC-9bus_CIM.cpp
	)
endif()

if(WITH_CIM AND WITH_SHMEM)
	set(CIM_SHMEM_SOURCES
	Shmem/Shmem_WSCC-9bus.cpp
	Shmem/Shmem_WSCC-9bus_Ctrl.cpp
	Shmem/Shmem_WSCC-9bus_CtrlDist.cpp
	)
endif()

if(WITH_PYTHON)
	list(APPEND LIBRARIES ${PYTHON_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
endif()

foreach(SOURCE ${CIRCUIT_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES} ${SHMEM_SOURCES} ${RT_SOURCES} ${CIM_SOURCES} ${CIM_SHMEM_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()

