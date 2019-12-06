set(LIBRARIES "dpsim")

if (NOT WIN32)
	# needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# targets
set(CIRCUIT_SOURCES
	# Dynamic phasor examples
	Circuits/DP_VS_RL1.cpp
	Circuits/DP_Circuits.cpp
	Circuits/DP_Basics_DP_Sims.cpp	
	Circuits/DP_PiLine.cpp
	Circuits/DP_DecouplingLine.cpp
	Circuits/DP_Diakoptics.cpp
	#Circuits/DP_VS_Trafo_R.cpp
	#Circuits/DP_ResVS_RL_Switch.cpp
	
	# EMT examples	
	Circuits/EMT_CS_RL1.cpp
	Circuits/EMT_VS_RL1.cpp
	Circuits/EMT_Circuits.cpp
	Circuits/DP_Basics_EMT_Sims.cpp	
	#Circuits/EMT_ResVS_RL_Switch.cpp

	# EMT and DP studies
	Circuits/DP_EMT_RL_SourceStep.cpp
)

set(SYNCGEN_SOURCES	
	Components/DP_SynGenDq7odTrapez_SteadyState.cpp
	Components/DP_SynGenDq7odTrapez_ThreePhFault.cpp
	Components/DP_SynGenTrStab_SteadyState.cpp
	Components/DP_SynGenTrStab_LoadStep.cpp
	Components/DP_EMT_SynGenDq7odTrapez_SteadyState.cpp
	Components/DP_EMT_SynGenDq7odTrapez_ThreePhFault.cpp
	Components/DP_EMT_SynGenDq7odTrapez_LoadStep.cpp
)

set(INVERTER_SOURCES
	Components/DP_Inverter_Grid.cpp
	Components/DP_Inverter_Grid_Parallel_FreqSplit.cpp
	Components/DP_Inverter_Grid_Sequential_FreqSplit.cpp	
)

if(WITH_SUNDIALS)
	list(APPEND SYNCGEN_SOURCES
		Components/DP_SynGenDq7odODE_SteadyState.cpp
		Components/DP_SynGenDq7odODE_ThreePhFault.cpp
		Components/DP_Multimachine_DQ_Parallel.cpp
		Components/DP_EMT_SynGenDq7odODE_SteadyState.cpp
		Components/DP_EMT_SynGenDq7odODE_ThreePhFault.cpp
		Components/DP_EMT_SynGenDq7odODE_LoadStep.cpp
	)

	set(DAE_SOURCES
		DAE/DAE_DP_test.cpp
	)
endif()

if(WITH_RT)
	set(RT_SOURCES
		RealTime/RT_DP_CS_R1.cpp
		RealTime/RT_DP_VS_RL2.cpp
	)
endif()

if(WITH_SHMEM)
	list(APPEND LIBRARIES ${VILLASNODE_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})

	set(SHMEM_SOURCES
		Shmem/ShmemExample.cpp
		Shmem/ShmemDistributedReference.cpp
		Shmem/ShmemDistributedDirect.cpp
		Shmem/ShmemDistributedVillas.cpp
		Shmem/ShmemControllableSource.cpp
		Shmem/ShmemControllableFiltSource.cpp
		Shmem/Shmem_CIGRE_MV_PowerFlowTest.cpp
		Shmem/Shmem_CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
	)
endif()

if(WITH_CIM)
	list(APPEND LIBRARIES ${CIMPP_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})

	set(CIM_SOURCES
		CIM/dpsim-cim.cpp	
		CIM/WSCC-9bus_CIM.cpp
		CIM/WSCC-9bus_CIM_Dyn.cpp
		CIM/WSCC-9bus_CIM_Dyn_Switch.cpp

		# powerflow example		
		CIM/Slack_Trafo_Load.cpp
		CIM/Slack_TrafoTapChanger_Load.cpp
		CIM/CIGRE_MV_PowerFlowTest.cpp
		CIM/CIGRE_MV_PowerFlowTest_LoadProfiles.cpp

		CIM/IEEE_LV_PowerFlowTest.cpp
		CIM/IEEE_LV_PowerFlowTest_LoadProfiles.cpp
		
		CIM/case145.cpp
		CIM/case300.cpp
		CIM/case9.cpp
		CIM/case14.cpp		

		CIM/WSCC_9bus_mult_decoupled.cpp
		CIM/WSCC_9bus_mult_coupled.cpp
		CIM/WSCC_9bus_mult_diakoptics.cpp
	)

	if(WITH_RT)
		list(APPEND RT_SOURCES
			# The Loadflow example needs CIM++ and RT
			RealTime/RT_CIGRE_MV_PowerFlowTest.cpp
		)
	endif()
endif()

if(WITH_CIM AND WITH_SHMEM)
	set(CIM_SHMEM_SOURCES
		Shmem/Shmem_WSCC-9bus.cpp
		Shmem/Shmem_WSCC-9bus_Ctrl.cpp
		Shmem/Shmem_WSCC-9bus_CtrlDist.cpp
	)
endif()

if(WITH_PYTHON)
	#list(APPEND LIBRARIES ${PYTHON_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
endif()

if(WITH_OPENMP)
	list(APPEND DPSIM_CXX_FLAGS ${OpenMP_CXX_FLAGS})
	list(APPEND LIBRARIES ${OpenMP_CXX_FLAGS})
endif()

foreach(SOURCE ${CIRCUIT_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES} ${SHMEM_SOURCES} ${RT_SOURCES} ${CIM_SOURCES} ${CIM_SOURCES_POSIX} ${CIM_SHMEM_SOURCES} ${DAE_SOURCES} ${INVERTER_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()

if(WITH_SHMEM)
	add_executable(ShmemRealtimeExample Shmem/ShmemExample.cpp)
	target_link_libraries(ShmemRealtimeExample ${LIBRARIES})
	target_include_directories(ShmemRealtimeExample PRIVATE ${INCLUDE_DIRS})	
	target_compile_definitions(ShmemRealtimeExample PUBLIC REALTIME)
endif()

