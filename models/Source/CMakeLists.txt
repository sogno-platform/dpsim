add_library(cps STATIC
	Logger.cpp
	MathUtils.cpp
	Attribute.cpp
	TopologicalNode.cpp
	TopologicalTerminal.cpp
	SimNode.cpp
	SimPowerComp.cpp
	SystemTopology.cpp
	CSVReader.cpp
)

list(APPEND CPS_SOURCES
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
	DP/DP_Ph1_Inverter.cpp
	DP/DP_Ph1_AvVoltageSourceInverterDQ.cpp
	DP/DP_Ph1_NetworkInjection.cpp
	DP/DP_Ph1_varResSwitch.cpp

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
	EMT/EMT_Ph3_SynchronGeneratorDQ.cpp
	EMT/EMT_Ph3_SynchronGeneratorDQTrapez.cpp
	# EMT/EMT_Ph3_SynchronGeneratorDQSmpl.cpp
	# EMT/EMT_Ph3_SynchronGeneratorDQSmplCompSource.cpp
	# EMT/EMT_Ph3_SynchronGeneratorVBR.cpp
	# EMT/EMT_Ph3_SynchronGeneratorVBRStandalone.cpp
	EMT/EMT_Ph3_SynchronGeneratorTrStab.cpp

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
	SP/SP_Ph1_Transformer3W.cpp
	SP/SP_Ph1_SolidStateTransformer.cpp
	SP/SP_Ph1_Shunt.cpp
	SP/SP_Ph1_SynchronGenerator.cpp
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
	Signal/PLL.cpp
	Signal/Integrator.cpp
	Signal/PowerControllerVSI.cpp
	Signal/SineWaveGenerator.cpp
	Signal/SignalGenerator.cpp
	Signal/FrequencyRampGenerator.cpp
	Signal/CosineFMGenerator.cpp
)

if(WITH_CIM)
	list(APPEND CPS_SOURCES CIM/Reader.cpp)

	list(APPEND CPS_INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})
	list(APPEND CPS_LIBRARIES ${CIMPP_LIBRARIES})
endif()

if(WITH_PYTHON)
	list(APPEND CPS_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
	list(APPEND CPS_LIBRARIES ${PYTHON_LIBRARIES})

if(WITH_NUMPY)
	list(APPEND CPS_SOURCES AttributeNumPy.cpp)
	list(APPEND CPS_INCLUDE_DIRS ${NUMPY_INCLUDE_DIRS})
endif()
endif()

if(WITH_GRAPHVIZ)
	list(APPEND CPS_SOURCES Graph.cpp)

	list(APPEND CPS_INCLUDE_DIRS ${GRAPHVIZ_INCLUDE_DIRS})
	list(APPEND CPS_LIBRARIES ${GRAPHVIZ_LIBRARIES})
endif()

if(WITH_SUNDIALS)
	list(APPEND CPS_SOURCES DP/DP_Ph3_SynchronGeneratorDQODE.cpp)
	list(APPEND CPS_SOURCES EMT/EMT_Ph3_SynchronGeneratorDQODE.cpp)
	list(APPEND CPS_INCLUDE_DIRS ${SUNDIALS_INCLUDE_DIRS})
	list(APPEND CPS_LIBRARIES ${SUNDIALS_LIBRARIES})
endif()

if(WITH_GSL)
	list(APPEND CPS_INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
	list(APPEND CPS_LIBRARIES ${GSL_LIBRARIES})
endif()

target_link_libraries(cps PUBLIC ${CPS_LIBRARIES})
target_include_directories(cps PUBLIC ${CPS_INCLUDE_DIRS})
target_sources(cps PRIVATE ${CPS_SOURCES})
target_compile_options(cps PUBLIC ${CPS_CXX_FLAGS})
