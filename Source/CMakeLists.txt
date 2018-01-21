add_library(dpsim
	IntegrationMethod.cpp
	Logger.cpp
	MathLibrary.cpp
	SystemModel.cpp
	Utilities.cpp
	Simulation.cpp
	FaultSimulation.cpp
	SynGenSimulation.cpp
	Component.cpp
)

list(APPEND SOURCES
	Components/Base_SynchronGenerator.cpp
	Components/DP_Capacitor.cpp
	Components/DP_CurrentSource.cpp
	Components/DP_Inductor.cpp
	Components/DP_Inductor_Interfaced.cpp
	Components/DP_Line_Pi.cpp
	Components/DP_Line_Rx.cpp
	Components/DP_Load_PQ.cpp
	Components/DP_Resistor.cpp
	Components/DP_SynchronGenerator.cpp
	Components/DP_SynchronGenerator_VBR.cpp
	Components/DP_SynchronGenerator_VBR_Simplified.cpp
	Components/DP_Transformer.cpp
	Components/DP_Transformer_Ideal.cpp
	Components/DP_VoltageSource.cpp
	Components/DP_VoltageSource_Freq.cpp
	Components/DP_VoltageSource_Norton.cpp
	Components/EMT_Capacitor.cpp
	Components/EMT_CurrentSource.cpp
	Components/EMT_Inductor.cpp
	Components/EMT_Resistor.cpp
	Components/EMT_SynchronGenerator.cpp
	Components/EMT_SynchronGenerator_Simplified.cpp
	Components/EMT_SynchronGenerator_Simplified_CurrentSource.cpp
	Components/EMT_SynchronGenerator_VBR.cpp
	Components/EMT_VoltageSource.cpp
	Components/EMT_VoltageSource_Freq.cpp
	Components/EMT_VoltageSource_Norton.cpp
	Components/Exciter.cpp
	Components/TurbineGovernor.cpp
	Components/EMT_SynchronGenerator_VBR_New.cpp
	Components/DP_SynchronGenerator_VBR_New.cpp
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
		ExternalInterface.cpp
		ShmemInterface.cpp
	)
endif()

if(WITH_PYTHON)
	add_subdirectory(Python)
endif()

if(WITH_CIM)
	list(APPEND SOURCES CIM/Reader.cpp)
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
