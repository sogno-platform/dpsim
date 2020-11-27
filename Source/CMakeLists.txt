set(DPSIM_SOURCES
	Simulation.cpp
	RealTimeSimulation.cpp
	MNASolver.cpp
	MNASolverSysRecomp.cpp
	PFSolver.cpp
	PFSolverPowerPolar.cpp
	Utils.cpp
	Timer.cpp
	Event.cpp
	DataLogger.cpp
	Scheduler.cpp
	SequentialScheduler.cpp
	ThreadScheduler.cpp
	ThreadLevelScheduler.cpp
	ThreadListScheduler.cpp
	DiakopticsSolver.cpp
)

list(APPEND DPSIM_LIBRARIES cps)

if(WITH_RT AND HAVE_TIMERFD)
	list(APPEND DPSIM_LIBRARIES "-lrt")
endif()

if(WITH_SUNDIALS)
	list(APPEND DPSIM_SOURCES DAESolver.cpp)
	#For ODE-Solver class:
    list(APPEND DPSIM_SOURCES ODESolver.cpp)
	list(APPEND DPSIM_INCLUDE_DIRS ${SUNDIALS_INCLUDE_DIRS})
	list(APPEND DPSIM_LIBRARIES ${SUNDIALS_LIBRARIES})
endif()

if(WITH_GSL)
	list(APPEND DPSIM_INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
	list(APPEND DPSIM_LIBRARIES ${GSL_LIBRARIES})
endif()

if(WITH_PYTHON)
	list(APPEND DPSIM_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
	#list(APPEND DPSIM_LIBRARIES ${PYTHON_LIBRARIES})

	add_subdirectory(Python)
endif()

if(WITH_SHMEM)
	list(APPEND DPSIM_SOURCES
		InterfaceShmem.cpp
		PthreadPoolScheduler.cpp
	)

	list(APPEND DPSIM_LIBRARIES ${VILLASNODE_LIBRARIES} pthread)
	list(APPEND DPSIM_INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})
endif()

if(WITH_CUDA)
	list(APPEND DPSIM_SOURCES
		MNASolverGpu.cpp
	)

	list(APPEND DPSIM_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
	list(APPEND DPSIM_LIBRARIES ${CUDA_LIBRARIES} ${CUDA_cusolver_LIBRARY})
endif()

if(WITH_OPENMP)
	list(APPEND DPSIM_SOURCES
		OpenMPLevelScheduler.cpp
	)

	list(APPEND DPSIM_CXX_FLAGS ${OpenMP_CXX_FLAGS})
	list(APPEND DPSIM_LIBRARIES ${OpenMP_CXX_FLAGS})
endif()

add_library(dpsim ${DPSIM_SOURCES})
target_link_libraries(dpsim PUBLIC ${DPSIM_LIBRARIES})
target_include_directories(dpsim PUBLIC ${DPSIM_INCLUDE_DIRS} ${CPS_LIBRARIES})
target_compile_options(dpsim PUBLIC ${DPSIM_CXX_FLAGS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

install(TARGETS dpsim
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
)
