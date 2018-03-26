add_library(dpsim	
	SystemModel.cpp
	Utilities.cpp
	Simulation.cpp
	MNA_Solver.cpp
	FaultSimulation.cpp
	SynGenSimulation.cpp	
	CPowerSystems/Source/Logger.cpp
	CPowerSystems/Source/MathUtils.cpp	
	CPowerSystems/Source/Attribute.cpp
	CPowerSystems/Source/Component.cpp
)

list(APPEND SOURCES	
	CPowerSystems/Source/Components/DP_Capacitor.cpp
	CPowerSystems/Source/Components/DP_CurrentSource.cpp
	CPowerSystems/Source/Components/DP_Inductor.cpp
	CPowerSystems/Source/Components/DP_Line_Pi.cpp
	CPowerSystems/Source/Components/DP_Line_Rx.cpp
	CPowerSystems/Source/Components/DP_Load_PQ.cpp
	CPowerSystems/Source/Components/DP_Resistor.cpp	
	CPowerSystems/Source/Components/DP_Transformer.cpp
	CPowerSystems/Source/Components/DP_VoltageSource.cpp
	CPowerSystems/Source/Components/DP_VoltageSource_Freq.cpp
	CPowerSystems/Source/Components/DP_VoltageSource_Norton.cpp
	CPowerSystems/Source/Components/EMT_Capacitor.cpp
	CPowerSystems/Source/Components/EMT_CurrentSource.cpp
	CPowerSystems/Source/Components/EMT_Inductor.cpp
	CPowerSystems/Source/Components/EMT_Resistor.cpp	
	CPowerSystems/Source/Components/EMT_VoltageSource.cpp
	CPowerSystems/Source/Components/EMT_VoltageSource_Freq.cpp
	CPowerSystems/Source/Components/EMT_VoltageSource_Norton.cpp
	CPowerSystems/Source/Components/Exciter.cpp
	CPowerSystems/Source/Components/TurbineGovernor.cpp
	CPowerSystems/Source/Components/DP_SynchronGenerator_Ideal.cpp
	CPowerSystems/Source/Components/Base_SynchronGenerator.cpp	
	CPowerSystems/Source/Components/DP_SynchronGenerator_Classic_Simplified.cpp
	CPowerSystems/Source/Components/DP_SynchronGenerator_Classic.cpp
	CPowerSystems/Source/Components/DP_SynchronGenerator_VBR.cpp
	CPowerSystems/Source/Components/DP_SynchronGenerator_VBR_UnitTest.cpp
	CPowerSystems/Source/Components/DP_SynchronGenerator_VBR_Simplified.cpp
	CPowerSystems/Source/Components/EMT_SynchronGenerator_Classic.cpp
	CPowerSystems/Source/Components/EMT_SynchronGenerator_Classic_Simplified.cpp
	CPowerSystems/Source/Components/EMT_SynchronGenerator_Classic_Simpl_CurrentComp.cpp
	CPowerSystems/Source/Components/EMT_SynchronGenerator_VBR.cpp
	CPowerSystems/Source/Components/EMT_SynchronGenerator_VBR_UnitTest.cpp
)

set(INCLUDE_DIRS
	${EIGEN3_INCLUDE_DIR}
)

if(NOT MSVC)
	set(LIBRARIES "-lstdc++fs")
endif()

if(NOT WIN32)
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -fPIC")
endif()

if(WITH_RT)
	list(APPEND LIBRARIES "-lrt")
	list(APPEND SOURCES RealTimeSimulation.cpp)
endif()

if(WITH_SHMEM)
	list(APPEND LIBRARIES ${VILLASNODE_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})

	list(APPEND SOURCES
		CPowerSystems/Source/Interfaces/ExternalInterface.cpp
		CPowerSystems/Source/Interfaces/ShmemInterface.cpp
	)
endif()

if(WITH_PYTHON)
	list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

	add_subdirectory(Python)
endif()

if(WITH_CIM)
	list(APPEND SOURCES CPowerSystems/Source/CIM/Reader.cpp)
	list(APPEND INCLUDE_DIRS ${CIMPP_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${CIMPP_LIBRARIES})
endif()

target_sources(dpsim PRIVATE ${SOURCES})
target_link_libraries(dpsim ${LIBRARIES})
target_include_directories(dpsim PRIVATE ${INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
configure_file(Config.h.in ${CMAKE_CURRENT_BINARY_DIR}/Config.h)

include(GNUInstallDirs)

install(TARGETS dpsim
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
)
