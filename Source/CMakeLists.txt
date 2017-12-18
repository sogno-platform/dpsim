add_library(dpsim
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
)

find_package(Eigen3 REQUIRED)
set(INCLUDE_DIRS
	${EIGEN3_INCLUDE_DIR}
)

if (NOT WIN32)
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif()

if(UNIX OR APPLE)
	set(PYTHON_MODULE_SUFFIX "so")
elseif(WIN32)
	set(PYTHON_MODULE_SUFFIX "pyd")
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

if (WITH_PYTHON)
	get_target_property(DPSIM_TYPE dpsim TYPE)
	if(${DPSIM_TYPE} STREQUAL SHARED_LIBRARY)
		list(APPEND SOURCES
			Python/Component.cpp
			Python/Module.cpp
			Python/Simulation.cpp
			Python/LoadCim.cpp
			Python/Interface.cpp
		)

		list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
		list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

		add_custom_target(link_dpsim ALL DEPENDS dpsim COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:dpsim> dpsim.${PYTHON_MODULE_SUFFIX})
		execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('platlib'), end='')" OUTPUT_VARIABLE PYTHON_PKGPATH)

		install(FILES $<TARGET_FILE:dpsim> DESTINATION ${PYTHON_PKGPATH} RENAME dpsim.${PYTHON_MODULE_SUFFIX})
	else()
		message(WARNING "In order to build DPsim as a Python module you must build it as a shared library: cmake -DBUILD_SHARED_LIBS=ON")
	endif()
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
