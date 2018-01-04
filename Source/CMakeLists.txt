add_library(dpsim SHARED
	IntegrationMethod.cpp
	Logger.cpp
	MathLibrary.cpp
	Simulation.cpp
	SystemModel.cpp
	Utilities.cpp
)

list(APPEND SOURCES
	Components/BaseComponent.cpp
	Components/CapacitorDP.cpp
	Components/CurrentSourceDP.cpp
	Components/Exciter.cpp
	Components/ExternalCurrentSource.cpp
	Components/ExternalVoltageSource.cpp
	Components/IdealTransformerDP.cpp
	Components/IdealVoltageSourceDP.cpp
	Components/IdealVoltageSourceEMT.cpp
	Components/InductorDP.cpp
	Components/InductorEMT.cpp
	Components/InterfacedInductorDP.cpp
	Components/PQLoadDP.cpp
	Components/PiLineDP.cpp
	Components/ResistorDP.cpp
	Components/ResistorEMT.cpp
	Components/RxLineDP.cpp
	Components/SimplifiedSynGenEMT.cpp
	Components/SimplifiedSynchronGeneratorEMT.cpp
	Components/SynchGenBase.cpp
	Components/SynchronGeneratorDP.cpp
	Components/SynchronGeneratorEMT.cpp
	Components/TransformerDP.cpp
	Components/VoltSourceResDP.cpp
	Components/VoltSourceResEMT.cpp
	Components/VoltSourceResFreqDP.cpp
	Components/VoltSourceResFreqEMT.cpp
	Components/VoltageBehindReachtanceDP.cpp
	Components/VoltageBehindReactanceEMT.cpp
	Components/TurbineGovernor.cpp
)

find_package(Eigen3 REQUIRED)
set(INCLUDE_DIRS
	${EIGEN3_INCLUDE_DIR}
)

set(LIBRARIES "-lstdc++fs")

if(NOT WIN32)
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif()

if(WITH_RT)
	list(APPEND LIBRARIES "-lrt")
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

install(TARGETS dpsim
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib/static
)
