set(LIBRARIES "dpsim")

if(NOT WIN32)
	# Needed so CIMParser and arabica (which are first compiled into static libs)
	# can be included in the .so
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# Targets
set(CIRCUIT_SOURCES
	# DP examples
	Circuits/DP_VS_RL1.cpp
	Circuits/DP_Circuits.cpp
	Circuits/DP_Basics_DP_Sims.cpp
	Circuits/DP_PiLine.cpp
	Circuits/DP_DecouplingLine.cpp
	Circuits/DP_Diakoptics.cpp
	Circuits/DP_VSI.cpp

	# DP examples with PF initialization
	Circuits/DP_Slack_PiLine_PQLoad_with_PF_Init.cpp
	Circuits/DP_Slack_PiLine_VSI_with_PF_Init.cpp
	Circuits/DP_Slack_PiLine_VSI_Ramp_with_PF_Init.cpp

	# Powerflow examples
	Circuits/PF_Slack_PiLine_PQLoad.cpp

	# EMT examples
	Circuits/EMT_CS_RL1.cpp
	Circuits/EMT_VS_RL1.cpp
	Circuits/EMT_Circuits.cpp
	Circuits/DP_Basics_EMT_Sims.cpp
	#Circuits/EMT_ResVS_RL_Switch.cpp
	Circuits/EMT_VSI.cpp
	Circuits/EMT_PiLine.cpp
	Circuits/EMT_Ph3_R3C1L1CS1_RC_vs_SSN.cpp
	Circuits/EMT_Ph3_RLC1VS1_RC_vs_SSN.cpp
	Circuits/EMT_Ph1_ExponentialDiode_test.cpp

	# EMT examples with PF initialization
	Circuits/EMT_Slack_PiLine_PQLoad_with_PF_Init.cpp
	Circuits/EMT_Slack_PiLine_VSI_with_PF_Init.cpp
	Circuits/EMT_Slack_PiLine_VSI_Ramp_with_PF_Init.cpp

	# SP examples
	Circuits/SP_Circuits.cpp
	Circuits/SP_PiLine.cpp
	Circuits/SP_Slack_PiLine_VSI_with_PF_Init.cpp
	Circuits/SP_Slack_PiLine_VSI_Ramp_with_PF_Init.cpp
	Circuits/SP_Slack_PiLine_PQLoad_with_PF_Init.cpp

	# Combined EMT/DP/SP examples
	Circuits/EMT_DP_SP_Slack.cpp
	Circuits/EMT_DP_SP_VS_Init.cpp
	Circuits/EMT_DP_SP_VS_RLC.cpp
	Circuits/DP_EMT_RL_SourceStep.cpp
	Circuits/EMT_DP_SP_Trafo.cpp
	Circuits/EMT_DP_SP_Slack_PiLine_PQLoad_FrequencyRamp_CosineFM.cpp

	#SMIB
	Circuits/SP_SynGenTrStab_SMIB_SteadyState.cpp
	Circuits/DP_SynGenTrStab_SMIB_SteadyState.cpp
	Circuits/EMT_SynGenTrStab_SMIB_SteadyState.cpp
	Circuits/SP_SynGenTrStab_SMIB_Fault.cpp
	Circuits/DP_SynGenTrStab_SMIB_Fault.cpp
	Circuits/EMT_SynGenDQ7odTrapez_SMIB_Fault.cpp
	Circuits/EMT_SynGenDQ7odTrapez_OperationalParams_SMIB_Fault.cpp
	Circuits/EMT_SynGenDQ7odTrapez_OperationalParams_SMIB_Fault_JsonSyngenParams.cpp
	Circuits/EMT_SynGenDQ7odTrapez_DP_SynGenTrStab_SMIB_Fault.cpp
	Circuits/EMT_SynGenTrStab_SMIB_Fault.cpp
	Circuits/SP_SynGenTrStab_SMIB_Fault_KundurExample1.cpp
	Circuits/EMT_SynGenVBR_OperationalParams_SMIB_Fault.cpp
	Circuits/EMT_SynGenVBR_SMIB_Fault.cpp
	Circuits/SP_ReducedOrderSG_SMIB_Fault.cpp
	Circuits/DP_ReducedOrderSG_SMIB_Fault.cpp
	Circuits/EMT_ReducedOrderSG_SMIB_Fault.cpp
	Circuits/SP_ReducedOrderSG_VBR_Load_Fault.cpp
	Circuits/DP_ReducedOrderSG_VBR_Load_Fault.cpp
	Circuits/EMT_ReducedOrderSG_VBR_Load_Fault.cpp
	Circuits/EMT_SynGen4OrderIter_SMIB_Fault.cpp

	# SMIB Reduced Order - Load step
	Circuits/SP_SMIB_ReducedOrderSG_LoadStep.cpp
	Circuits/DP_SMIB_ReducedOrderSG_LoadStep.cpp
	Circuits/EMT_SMIB_ReducedOrderSG_LoadStep.cpp
	Circuits/EMT_SMIB_ReducedOrderSGIterative_LoadStep.cpp
	Circuits/DP_SMIB_ReducedOrderSGIterative_LoadStep.cpp

	#3Bus System
	Circuits/SP_SynGenTrStab_3Bus_SteadyState.cpp
	Circuits/DP_SynGenTrStab_3Bus_SteadyState.cpp
	Circuits/DP_SynGenTrStab_3Bus_Fault.cpp
	Circuits/SP_SynGenTrStab_3Bus_Fault.cpp

)

set(SYNCGEN_SOURCES
	Components/DP_SynGenDq7odTrapez_SteadyState.cpp
	Components/DP_SynGenDq7odTrapez_ThreePhFault.cpp
	Components/DP_SynGenTrStab_SteadyState.cpp
	Components/DP_SynGenTrStab_LoadStep.cpp
	Components/DP_EMT_SynGenDq7odTrapez_SteadyState.cpp
	Components/DP_EMT_SynGenDq7odTrapez_ThreePhFault.cpp
	Components/DP_EMT_SynGenDq7odTrapez_LoadStep.cpp
	Components/DP_SP_SynGenTrStab_testModels.cpp
	Components/DP_SP_SynGenTrStab_testModels_doubleLine.cpp
	Components/EMT_SynchronGenerator9OrderDCIM_LoadStep_TurbineGovernor_Exciter.cpp
	Components/EMT_SynchronGenerator9OrderVBR_LoadStep_TurbineGovernor_Exciter.cpp
)

set(INVERTER_SOURCES
	Components/DP_Inverter_Grid.cpp
	Components/DP_Inverter_Grid_Parallel_FreqSplit.cpp
	Components/DP_Inverter_Grid_Sequential_FreqSplit.cpp
)

# Targets required for tests in the Jupyter Notebooks. This list is only for grouping the (already configured) targets, so every entry
# also has to appear in another list in this file.
list(APPEND TEST_SOURCES
	Circuits/SP_ReducedOrderSG_SMIB_Fault.cpp
	Circuits/DP_ReducedOrderSG_SMIB_Fault.cpp
	Circuits/EMT_ReducedOrderSG_SMIB_Fault.cpp
	Circuits/SP_ReducedOrderSG_VBR_Load_Fault.cpp
	Circuits/DP_ReducedOrderSG_VBR_Load_Fault.cpp
	Circuits/EMT_ReducedOrderSG_VBR_Load_Fault.cpp
	Circuits/SP_SynGenTrStab_SMIB_Fault_KundurExample1.cpp
	Circuits/EMT_SynGenDQ7odTrapez_OperationalParams_SMIB_Fault.cpp
	Components/EMT_SynchronGenerator9OrderDCIM_LoadStep_TurbineGovernor_Exciter.cpp
	Components/EMT_SynchronGenerator9OrderVBR_LoadStep_TurbineGovernor_Exciter.cpp
	Circuits/DP_Slack_PiLine_VSI_Ramp_with_PF_Init.cpp
	Circuits/EMT_SynGenVBR_OperationalParams_SMIB_Fault.cpp
	Circuits/EMT_SynGenVBR_SMIB_Fault.cpp
	Circuits/EMT_SynGenDQ7odTrapez_SMIB_Fault.cpp
	Circuits/EMT_Slack_PiLine_VSI_Ramp_with_PF_Init.cpp
	Circuits/EMT_DP_SP_Slack_PiLine_PQLoad_FrequencyRamp_CosineFM.cpp
	Circuits/DP_SMIB_ReducedOrderSG_LoadStep.cpp
	Circuits/DP_SMIB_ReducedOrderSGIterative_LoadStep.cpp
	Circuits/EMT_Ph3_R3C1L1CS1_RC_vs_SSN.cpp
	Circuits/EMT_Ph3_RLC1VS1_RC_vs_SSN.cpp
	Circuits/EMT_Ph1_ExponentialDiode_test.cpp
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

if(WITH_CIM)
	list(APPEND LIBRARIES libcimpp)

	set(CIM_SOURCES
		CIM/WSCC-9bus_CIM.cpp
		CIM/WSCC-9bus_CIM_Dyn.cpp
		CIM/WSCC-9bus_CIM_Dyn_Switch.cpp
		CIM/SP_WSCC-9bus_CIM_Dyn_Switch.cpp
		CIM/EMT_WSCC-9bus_IdealVS.cpp
		CIM/EMT_WSCC-9bus_IdealCS.cpp
		CIM/EMT_WSCC-9bus_FullOrderSG.cpp
		CIM/EMT_WSCC-9bus_VBR.cpp
		CIM/DP_WSCC-9bus_IdealVS.cpp

		# WSCC Reduced Order
		CIM/SP_WSCC9bus_SGReducedOrderVBR.cpp
		CIM/EMT_WSCC9bus_SGReducedOrderVBR.cpp
		CIM/DP_WSCC9bus_SGReducedOrderVBR.cpp
		CIM/DP_WSCC9bus_SGReducedOrderIter.cpp

		# PF(Power Flow) example
		CIM/Slack_Trafo_Load.cpp
		CIM/Slack_TrafoTapChanger_Load.cpp
		CIM/CIGRE_MV_PowerFlowTest.cpp
		CIM/CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
		CIM/IEEE_LV_PowerFlowTest.cpp

		# WSCC examples
		CIM/WSCC_9bus_mult_decoupled.cpp
		CIM/WSCC_9bus_mult_coupled.cpp
		CIM/WSCC_9bus_mult_diakoptics.cpp
		CIM/DP_WSCC_9bus_split_decoupled.cpp
		CIM/EMT_WSCC_9bus_split_decoupled.cpp

		# CIGRE MV examples
		CIM/PF_CIGRE_MV_withDG.cpp
		CIM/DP_CIGRE_MV_withoutDG.cpp
		CIM/DP_CIGRE_MV_withDG.cpp
		CIM/DP_CIGRE_MV_withDG_withLoadStep.cpp
		CIM/EMT_CIGRE_MV_withoutDG.cpp
		CIM/EMT_CIGRE_MV_withDG.cpp
		CIM/EMT_CIGRE_MV_withDG_withLoadStep.cpp
	)

	list(APPEND TEST_SOURCES
		CIM/CIGRE_MV_PowerFlowTest.cpp
		CIM/DP_WSCC-9bus_IdealVS.cpp
		CIM/EMT_WSCC-9bus_IdealCS.cpp
		CIM/EMT_WSCC-9bus_IdealVS.cpp
		CIM/SP_WSCC9bus_SGReducedOrderVBR.cpp
	)

	if(WITH_RT)
		list(APPEND RT_SOURCES
			# The Loadflow example needs CIM++ and RT
			RealTime/RT_CIGRE_MV_PowerFlowTest.cpp
		)
	endif()
endif()

if(WITH_OPENMP)
	list(APPEND DPSIM_CXX_FLAGS ${OpenMP_CXX_FLAGS})
	list(APPEND LIBRARIES ${OpenMP_CXX_FLAGS})
endif()

add_custom_target(tests)

foreach(SOURCE ${CIRCUIT_SOURCES} ${SYNCGEN_SOURCES} ${VARFREQ_SOURCES} ${RT_SOURCES} ${CIM_SOURCES} ${CIM_SOURCES_POSIX} ${DAE_SOURCES} ${INVERTER_SOURCES})
	get_filename_component(TARGET ${SOURCE} NAME_WE)

	add_executable(${TARGET} ${SOURCE})

	target_link_libraries(${TARGET} ${LIBRARIES})
	target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIRS})
	target_compile_options(${TARGET} PUBLIC ${DPSIM_CXX_FLAGS})
endforeach()

foreach(TEST_SOURCE ${TEST_SOURCES})
	get_filename_component(TARGET ${TEST_SOURCE} NAME_WE)
	add_dependencies(tests ${TARGET})
endforeach()

add_subdirectory(cim_graphviz)
add_subdirectory(signals)
