add_library(dpsim-models STATIC
	Logger.cpp
	MathUtils.cpp
	MNAStampUtils.cpp
	Attribute.cpp
	TopologicalNode.cpp
	TopologicalTerminal.cpp
	SimNode.cpp
	SimPowerComp.cpp
	MNASimPowerComp.cpp
	CompositePowerComp.cpp
	SystemTopology.cpp
	CSVReader.cpp
)

list(APPEND MODELS_SOURCES
	Base/Base_ReducedOrderSynchronGenerator.cpp
	Base/Base_SynchronGenerator.cpp
	Base/Base_AvVoltageSourceInverterDQ.cpp
	Base/Base_AvVoltageSourceInverterDQWithStateSpace.cpp

	DP/DP_Ph1_Capacitor.cpp
	DP/DP_Ph1_CurrentSource.cpp
	DP/DP_Ph1_Inductor.cpp
	DP/DP_Ph1_PiLine.cpp
	DP/DP_Ph1_RxLine.cpp
	DP/DP_Ph1_RXLoad.cpp
	DP/DP_Ph1_RXLoadSwitch.cpp
	DP/DP_Ph1_PQLoadCS.cpp
	DP/DP_Ph1_Resistor.cpp
	DP/DP_Ph1_Transformer.cpp
	DP/DP_Ph1_VoltageSource.cpp
	DP/DP_Ph1_VoltageSourceRamp.cpp
	DP/DP_Ph1_VoltageSourceNorton.cpp
	DP/DP_Ph1_Switch.cpp
	DP/DP_Ph1_SynchronGeneratorIdeal.cpp
	DP/DP_Ph1_SynchronGeneratorTrStab.cpp
	DP/DP_Ph1_ReducedOrderSynchronGeneratorVBR.cpp
	DP/DP_Ph1_SynchronGenerator3OrderVBR.cpp
	DP/DP_Ph1_SynchronGenerator4OrderVBR.cpp
	DP/DP_Ph1_SynchronGenerator4OrderPCM.cpp
	DP/DP_Ph1_SynchronGenerator4OrderTPM.cpp
	DP/DP_Ph1_SynchronGenerator5OrderVBR.cpp
	DP/DP_Ph1_SynchronGenerator6OrderPCM.cpp
	DP/DP_Ph1_SynchronGenerator6aOrderVBR.cpp
	DP/DP_Ph1_SynchronGenerator6bOrderVBR.cpp
	DP/DP_Ph1_Inverter.cpp
	DP/DP_Ph1_AvVoltageSourceInverterDQ.cpp
	DP/DP_Ph1_NetworkInjection.cpp
	DP/DP_Ph1_varResSwitch.cpp
	DP/DP_Ph1_DPDQInterface.cpp

	DP/DP_Ph3_VoltageSource.cpp
	DP/DP_Ph3_Capacitor.cpp
	DP/DP_Ph3_Inductor.cpp
	DP/DP_Ph3_Resistor.cpp
	DP/DP_Ph3_SeriesResistor.cpp
	DP/DP_Ph3_SeriesSwitch.cpp
	DP/DP_Ph3_SynchronGeneratorDQ.cpp
	DP/DP_Ph3_SynchronGeneratorDQTrapez.cpp
	# DP/DP_Ph3_SynchronGeneratorDQSmpl.cpp
	# DP/DP_Ph3_SynchronGeneratorVBR.cpp
	# DP/DP_Ph3_SynchronGeneratorVBRStandalone.cpp

	EMT/EMT_Ph1_Capacitor.cpp
	EMT/EMT_Ph1_CurrentSource.cpp
	EMT/EMT_Ph1_Inductor.cpp
	EMT/EMT_Ph1_Resistor.cpp
	EMT/EMT_Ph1_VoltageSource.cpp
	EMT/EMT_Ph1_VoltageSourceRamp.cpp
	EMT/EMT_Ph1_VoltageSourceNorton.cpp
	EMT/EMT_Ph1_Switch.cpp
	EMT/EMT_Ph1_ExponentialDiode.cpp

	EMT/EMT_Ph3_CurrentSource.cpp
	EMT/EMT_Ph3_VoltageSource.cpp
	EMT/EMT_Ph3_Inductor.cpp
	EMT/EMT_Ph3_Capacitor.cpp
	EMT/EMT_Ph3_AvVoltageSourceInverterDQ.cpp
	EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.cpp
	EMT/EMT_Ph3_Resistor.cpp
	EMT/EMT_Ph3_SeriesResistor.cpp
	EMT/EMT_Ph3_SeriesSwitch.cpp
	EMT/EMT_Ph3_VoltageSourceNorton.cpp
	EMT/EMT_Ph3_PiLine.cpp
	EMT/EMT_Ph3_RxLine.cpp
	EMT/EMT_Ph3_RXLoad.cpp
	EMT/EMT_Ph3_NetworkInjection.cpp
	EMT/EMT_Ph3_Transformer.cpp
	EMT/EMT_Ph3_Switch.cpp
	EMT/EMT_Ph3_ReducedOrderSynchronGeneratorVBR.cpp
	EMT/EMT_Ph3_SynchronGenerator3OrderVBR.cpp
	EMT/EMT_Ph3_SynchronGenerator4OrderVBR.cpp
	EMT/EMT_Ph3_SynchronGenerator5OrderVBR.cpp
	EMT/EMT_Ph3_SynchronGenerator6aOrderVBR.cpp
	EMT/EMT_Ph3_SynchronGenerator6bOrderVBR.cpp
	EMT/EMT_Ph3_SynchronGeneratorDQ.cpp
	EMT/EMT_Ph3_SynchronGenerator4OrderPCM.cpp
	EMT/EMT_Ph3_SynchronGeneratorDQTrapez.cpp
	# EMT/EMT_Ph3_SynchronGeneratorDQSmpl.cpp
	# EMT/EMT_Ph3_SynchronGeneratorDQSmplCompSource.cpp
	EMT/EMT_Ph3_SynchronGeneratorVBR.cpp
	EMT/EMT_Ph3_SynchronGeneratorIdeal.cpp
	# EMT/EMT_Ph3_SynchronGeneratorVBRStandalone.cpp
	EMT/EMT_Ph3_SynchronGeneratorTrStab.cpp
	EMT/EMT_Ph3_SSN_Capacitor.cpp
	EMT/EMT_Ph3_SSN_Inductor.cpp
	EMT/EMT_Ph3_SSN_Full_Serial_RLC.cpp

	SP/SP_Ph1_VoltageSource.cpp
	SP/SP_Ph1_Capacitor.cpp
	SP/SP_Ph1_Inductor.cpp
	SP/SP_Ph1_Resistor.cpp
	SP/SP_Ph1_AvVoltageSourceInverterDQ.cpp
	SP/SP_Ph1_RXLine.cpp
	SP/SP_Ph1_VoltageSourceInverter.cpp
	SP/SP_Ph1_PiLine.cpp
	SP/SP_Ph1_Load.cpp
	SP/SP_Ph1_Switch.cpp
	SP/SP_Ph1_Transformer.cpp
	SP/SP_Ph1_SolidStateTransformer.cpp
	SP/SP_Ph1_Shunt.cpp
	SP/SP_Ph1_SynchronGenerator.cpp
	SP/SP_Ph1_ReducedOrderSynchronGeneratorVBR.cpp
	SP/SP_Ph1_SynchronGenerator3OrderVBR.cpp
	SP/SP_Ph1_SynchronGenerator4OrderVBR.cpp
	SP/SP_Ph1_SynchronGenerator5OrderVBR.cpp
	SP/SP_Ph1_SynchronGenerator6aOrderVBR.cpp
	SP/SP_Ph1_SynchronGenerator6bOrderVBR.cpp
	SP/SP_Ph1_PQNode.cpp
	SP/SP_Ph1_PVNode.cpp
	SP/SP_Ph1_VDNode.cpp
	SP/SP_Ph1_NetworkInjection.cpp
	SP/SP_Ph1_SynchronGeneratorTrStab.cpp
	SP/SP_Ph1_varResSwitch.cpp

	SP/SP_Ph3_Capacitor.cpp
	SP/SP_Ph3_Inductor.cpp
	SP/SP_Ph3_Resistor.cpp
	SP/SP_Ph3_VoltageSource.cpp

	Signal/DecouplingLine.cpp
	Signal/DecouplingLineEMT.cpp
	Signal/Exciter.cpp
	Signal/FIRFilter.cpp
	Signal/TurbineGovernor.cpp
	Signal/TurbineGovernorType1.cpp
	Signal/PLL.cpp
	Signal/Integrator.cpp
	Signal/PowerControllerVSI.cpp
	Signal/SineWaveGenerator.cpp
	Signal/SignalGenerator.cpp
	Signal/FrequencyRampGenerator.cpp
	Signal/CosineFMGenerator.cpp
	Signal/DCGenerator.cpp
)

if(WITH_CIM)
	list(APPEND MODELS_SOURCES CIM/Reader.cpp)

	list(APPEND MODELS_LIBRARIES libcimpp)
endif()

if(WITH_GRAPHVIZ)
	list(APPEND MODELS_SOURCES Graph.cpp)

	list(APPEND MODELS_INCLUDE_DIRS ${GRAPHVIZ_INCLUDE_DIRS})
	list(APPEND MODELS_LIBRARIES ${GRAPHVIZ_LIBRARIES})
endif()

if(WITH_SUNDIALS)
	list(APPEND MODELS_SOURCES DP/DP_Ph3_SynchronGeneratorDQODE.cpp)
	list(APPEND MODELS_SOURCES EMT/EMT_Ph3_SynchronGeneratorDQODE.cpp)
	list(APPEND MODELS_INCLUDE_DIRS ${SUNDIALS_INCLUDE_DIRS})
	list(APPEND MODELS_LIBRARIES ${SUNDIALS_LIBRARIES})
endif()

if(WITH_GSL)
	list(APPEND MODELS_INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
	list(APPEND MODELS_LIBRARIES ${GSL_LIBRARIES})
endif()

if(WITH_VILLAS)
	list(APPEND MODELS_SOURCES DP/DP_Ph1_ProfileVoltageSource.cpp)
endif()

target_link_libraries(dpsim-models PUBLIC ${MODELS_LIBRARIES})
target_include_directories(dpsim-models PUBLIC ${MODELS_INCLUDE_DIRS})
target_sources(dpsim-models PRIVATE ${MODELS_SOURCES})
target_compile_options(dpsim-models PUBLIC ${MODELS_CXX_FLAGS})
