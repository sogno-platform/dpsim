set(LIBRARIES "dpsim")

if (NOT WIN32)
	# needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "-fPIC")

	if(CXX_SUPPORTS_PROF AND WITH_PROFILING)
       set(CMAKE_CXX_FLAGS "-fPIC -O0 -pg")
	endif() 
endif()

# targets
set(CIRCUIT_SOURCES
	# Dynamic phasor examples
	Circuits/DP_VS_R1.cpp
	Circuits/DP_CS_R1.cpp
	Circuits/DP_VS_RL1.cpp
	Circuits/DP_VS_RC1.cpp	
	Circuits/DP_CS_R2CL.cpp
	Circuits/DP_VS_CS_R4.cpp
	Circuits/DP_VS_R2L3.cpp

	#Circuits/DP_VS_PiLine_R.cpp
	#Circuits/DP_VS_Trafo_R.cpp
	#Circuits/DP_ResVS_RL_Switch.cpp	

	# EMT examples
	Circuits/EMT_VS_R1.cpp
	Circuits/EMT_VS_RC1.cpp
	Circuits/EMT_CS_R1.cpp
	Circuits/EMT_VS_RL1.cpp
	Circuits/EMT_CS_R2CL.cpp
	Circuits/EMT_VS_R2L3.cpp
	Circuits/EMT_VS_CS_R4_AC.cpp
	Circuits/EMT_VS_CS_R4_DC.cpp	
	
	#Circuits/EMT_ResVS_RL_Switch.cpp
)

set(SYNCGEN_SOURCES
	SynchronGenerator/DP_SynGen_dq_SteadyState.cpp
	SynchronGenerator/DP_SynGen_TrStab_SteadyState.cpp
	SynchronGenerator/DP_SynGen_TrStab_LoadStep.cpp
	#SynchronGenerator/DP_SynchronGenerator_BalancedResLoad.cpp
	#SynchronGenerator/DP_SynchronGenerator_ExciterAndTurbine.cpp
	#SynchronGenerator/DP_SynchronGenerator_SimpThreePhaseFault.cpp
	#SynchronGenerator/DP_SynchronGenerator_ThreePhaseFault.cpp
	#SynchronGenerator/DP_SynchronGenerator_VBR.cpp
	#SynchronGenerator/DP_SynchronGenerator_Simplified_ThreePhaseFault.cpp
	#SynchronGenerator/EMT_SynchronGenerator_BalancedResLoad.cpp
	#SynchronGenerator/EMT_SynchronGenerator_ExciterAndTurbine.cpp
	#SynchronGenerator/EMT_SynchronGenerator_PhaseToPhaseFault.cpp
	#SynchronGenerator/EMT_SynchronGenerator_ThreePhaseFault.cpp
	#SynchronGenerator/EMT_SynchronGenerator_VBR_ThreePhaseFault.cpp
	#SynchronGenerator/EMT_SynchronGenerator_Simplified_ThreePhaseFault.cpp
	#SynchronGenerator/EMT_Multimachine.cpp
)

if(WITH_SUNDIALS)
	list(APPEND SYNCGEN_SOURCES
		SynchronGenerator/DP_SynGen_dq_ThreePhFault.cpp
	)
endif()

set(VARFREQ_SOURCES
	#VariableTimeStep/RXLine_LoadStep_FreqStep_1.cpp
	#VariableTimeStep/RXLine_LoadStep_FreqStep_2.cpp
)

if(WITH_SUNDIALS)
	set(DAE_SOURCES
		DAE/DAE_DP_test.cpp
	)
endif()

if(WITH_RT)
	set(RT_SOURCES
		RealTime/RT_DP_CS_R_1.cpp
		RealTime/RT_DP_ResVS_RL1.cpp
		RealTime/RT_CIGRE_MV_PowerFlowTest.cpp
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
		Shmem/ShmemRealTime.cpp
		Shmem/Shmem_CIGRE_MV_PowerFlowTest.cpp
		Shmem/Shmem_CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
	)
endif()

if(WITH_CIM)
	list(APPEND LIBRARIES ${CIMPP_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})

	if (NOT WIN32)
		set(CIM_SOURCES_POSIX
			CIM/dpsim-cim.cpp
		)
	endif()

	set(CIM_SOURCES
		
		CIM/WSCC-9bus_CIM.cpp
		CIM/WSCC-9bus_CIM_Dyn.cpp
		CIM/WSCC-9bus_CIM_Dyn_Switch.cpp

		# powerflow example
		CIM/CIGRE_MV_PowerFlowTest.cpp
		CIM/CIGRE_MV_PowerFlowTest_LoadProfiles.cpp

		CIM/IEEE_LV_PowerFlowTest.cpp
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

foreach(SOURCE ${CIRCUIT_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES} ${SHMEM_SOURCES} ${RT_SOURCES} ${CIM_SOURCES} ${CIM_SOURCES_POSIX} ${CIM_SHMEM_SOURCES} ${DAE_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()
