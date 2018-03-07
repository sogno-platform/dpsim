add_library(dpsim	
	SystemModel.cpp
	Utilities.cpp
	Simulation.cpp
	FaultSimulation.cpp
	SynGenSimulation.cpp
	CPowerSystems/Components/Attribute.cpp
	CPowerSystems/IntegrationMethod.cpp
	CPowerSystems/Logger.cpp
	CPowerSystems/MathLibrary.cpp
	CPowerSystems/Components/Component.cpp
	CPowerSystems/IntegrationMethod.cpp
	CPowerSystems/Logger.cpp
	CPowerSystems/MathLibrary.cpp
)

list(APPEND SOURCES
	CPowerSystems/Components/Base_SynchronGenerator.cpp
	CPowerSystems/Components/DP_Capacitor.cpp
	CPowerSystems/Components/DP_CurrentSource.cpp
	CPowerSystems/Components/DP_Inductor.cpp
	CPowerSystems/Components/DP_Inductor_Interfaced.cpp
	CPowerSystems/Components/DP_Line_Pi.cpp
	CPowerSystems/Components/DP_Line_Rx.cpp
	CPowerSystems/Components/DP_Load_PQ.cpp
	CPowerSystems/Components/DP_Resistor.cpp
	CPowerSystems/Components/DP_SynchronGenerator_Ideal.cpp
	CPowerSystems/Components/DP_SynchronGenerator.cpp
	CPowerSystems/Components/DP_SynchronGenerator_VBR.cpp
	CPowerSystems/Components/DP_SynchronGenerator_VBR_Simplified.cpp
	CPowerSystems/Components/DP_Transformer.cpp
	CPowerSystems/Components/DP_Transformer_Ideal.cpp
	CPowerSystems/Components/DP_VoltageSource.cpp
	CPowerSystems/Components/DP_VoltageSource_Freq.cpp
	CPowerSystems/Components/DP_VoltageSource_Norton.cpp
	CPowerSystems/Components/EMT_Capacitor.cpp
	CPowerSystems/Components/EMT_CurrentSource.cpp
	CPowerSystems/Components/EMT_Inductor.cpp
	CPowerSystems/Components/EMT_Resistor.cpp
	CPowerSystems/Components/EMT_SynchronGenerator.cpp
	CPowerSystems/Components/EMT_SynchronGenerator_Simplified.cpp
	CPowerSystems/Components/EMT_SynchronGenerator_Simplified_CurrentSource.cpp
	CPowerSystems/Components/EMT_SynchronGenerator_VBR.cpp
	CPowerSystems/Components/EMT_VoltageSource.cpp
	CPowerSystems/Components/EMT_VoltageSource_Freq.cpp
	CPowerSystems/Components/EMT_VoltageSource_Norton.cpp
	CPowerSystems/Components/Exciter.cpp
	CPowerSystems/Components/TurbineGovernor.cpp
	CPowerSystems/Components/EMT_SynchronGenerator_VBR_New.cpp
	CPowerSystems/Components/DP_SynchronGenerator_VBR_New.cpp
	CPowerSystems/Components/DP_SynchronGenerator_Simplified.cpp
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
		CPowerSystems/ExternalInterface.cpp
		CPowerSystems/ShmemInterface.cpp
	)
endif()

if(WITH_PYTHON)
	list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

	add_subdirectory(Python)
endif()

if(WITH_CIM)
	list(APPEND SOURCES CPowerSystems/CIM/Reader.cpp)
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
