set(LIBRARIES "dpsim")

if (NOT WIN32)
	# needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
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

	# EMT and DP studies
	#VariableTimeStep/RXLine_LoadStep_FreqStep_1.cpp	
)

set(SYNCGEN_SOURCES
	Components/DP_SynGen_dq_SteadyState.cpp
	Components/DP_SynGen_TrStab_SteadyState.cpp
	Components/DP_SynGen_TrStab_LoadStep.cpp
)

if(WITH_SUNDIALS)
	list(APPEND SYNCGEN_SOURCES
		Components/DP_SynGen_dq_ThreePhFault.cpp
		Components/DP_Multimachine_DQ_Parallel.cpp
	)
endif()

if(WITH_SUNDIALS)
	set(DAE_SOURCES
		DAE/DAE_DP_test.cpp
	)
endif()

if(WITH_RT)
	set(RT_SOURCES
		RealTime/RT_DP_CS_R_1.cpp
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
		Shmem/ShmemDistributedVillas.cpp
		Shmem/ShmemControllableSource.cpp
		Shmem/ShmemControllableFiltSource.cpp
		Shmem/ShmemRealTime.cpp
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
		CIM/WSCC-9bus_CIM_mult.cpp
		CIM/WSCC-9bus_CIM_Dyn.cpp
		CIM/WSCC-9bus_CIM_Dyn_Switch.cpp

		# powerflow example
		CIM/CIGRE_MV_PowerFlowTest.cpp
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

if(WITH_OPENMP)
	list(APPEND DPSIM_CXX_FLAGS ${OpenMP_CXX_FLAGS})
	list(APPEND LIBRARIES ${OpenMP_CXX_FLAGS})
endif()

foreach(SOURCE ${CIRCUIT_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES} ${SHMEM_SOURCES} ${RT_SOURCES} ${CIM_SOURCES} ${CIM_SOURCES_POSIX} ${CIM_SHMEM_SOURCES} ${DAE_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()
