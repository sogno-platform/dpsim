set(SOURCES
	Simulation.cpp
	RealTimeSimulation.cpp
	MNASolver.cpp
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

list(APPEND LIBRARIES cps)

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

if(WITH_RT AND HAVE_TIMERFD)
	list(APPEND LIBRARIES "-lrt")
endif()

if(WITH_SUNDIALS)
	list(APPEND SOURCES DAESolver.cpp)
	#For ODE-Solver class:
    list(APPEND SOURCES ODESolver.cpp)
	list(APPEND INCLUDE_DIRS ${SUNDIALS_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${SUNDIALS_LIBRARIES})
endif()

if(WITH_GSL)
	list(APPEND INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${GSL_LIBRARIES})
endif()

if(WITH_PYTHON)
	list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
	#list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

	add_subdirectory(Python)
endif()

if(WITH_SHMEM)
	list(APPEND SOURCES
		Interface.cpp
		PthreadPoolScheduler.cpp
	)

	list(APPEND LIBRARIES ${VILLASNODE_LIBRARIES} pthread)
	list(APPEND INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})
endif()

if(WITH_OPENMP)
	list(APPEND SOURCES
		OpenMPLevelScheduler.cpp
	)

	list(APPEND DPSIM_CXX_FLAGS ${OpenMP_CXX_FLAGS})
	list(APPEND LIBRARIES ${OpenMP_CXX_FLAGS})
endif()

add_library(dpsim ${SOURCES})
target_link_libraries(dpsim ${LIBRARIES})
target_include_directories(dpsim PUBLIC ${INCLUDE_DIRS})
target_compile_options(dpsim PUBLIC ${DPSIM_CXX_FLAGS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

install(TARGETS dpsim
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
)
