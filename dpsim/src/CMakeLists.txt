set(DPSIM_SOURCES
	Simulation.cpp
	RealTimeSimulation.cpp
	MNASolver.cpp
	MNASolverDirect.cpp
	IterativeMnaSolverDirect.cpp
	DenseLUAdapter.cpp
	SparseLUAdapter.cpp
	DirectLinearSolverConfiguration.cpp
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
	Interface.cpp
	InterfaceQueued.cpp
)

list(APPEND DPSIM_LIBRARIES
	dpsim-models
	readerwriterqueue
	filesystem
)

if(WITH_MNASOLVERPLUGIN)
	list(APPEND DPSIM_LIBRARIES ${CMAKE_DL_LIBS})
	list(APPEND DPSIM_SOURCES MNASolverPlugin.cpp)
endif()

if(WITH_RT AND HAVE_TIMERFD)
	list(APPEND DPSIM_LIBRARIES "-lrt")
endif()

if(WITH_SUNDIALS)
	list(APPEND DPSIM_SOURCES DAESolver.cpp)

	# For ODE-Solver class:
	list(APPEND DPSIM_SOURCES ODESolver.cpp)
	list(APPEND DPSIM_INCLUDE_DIRS ${SUNDIALS_INCLUDE_DIRS})
	list(APPEND DPSIM_LIBRARIES ${SUNDIALS_LIBRARIES})
endif()

if(WITH_GSL)
	list(APPEND DPSIM_INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
	list(APPEND DPSIM_LIBRARIES ${GSL_LIBRARIES})
endif()

if(WITH_JSON)
	list(APPEND DPSIM_LIBRARIES nlohmann_json::nlohmann_json)
endif()

if(WITH_KLU)
	list(APPEND DPSIM_LIBRARIES klu)
	list(APPEND DPSIM_SOURCES
		KLUAdapter.cpp
	)
endif()

if(WITH_CUDA)
	list(APPEND DPSIM_INCLUDE_DIRS
		${CUDA_INCLUDE_DIRS}
		${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
	)

	list(APPEND DPSIM_SOURCES GpuDenseAdapter.cpp)

	list(APPEND DPSIM_LIBRARIES
		${CUDA_LIBRARIES}
		${CUDA_cusolver_LIBRARY}
	)

	if(WITH_MAGMA)
		list(APPEND DPSIM_SOURCES
			GpuMagmaAdapter.cpp
			)
		list(APPEND DPSIM_LIBRARIES ${MAGMA_LIBRARIES})
		list(APPEND DPSIM_INCLUDE_DIRS ${MAGMA_INCLUDE_DIR})
	endif()
	if(WITH_CUDA_SPARSE)
		list(APPEND DPSIM_SOURCES
			GpuSparseAdapter.cpp
		)

		list(APPEND DPSIM_LIBRARIES ${CUDA_cusparse_LIBRARY})
	endif()
endif()

if(WITH_OPENMP)
	list(APPEND DPSIM_SOURCES OpenMPLevelScheduler.cpp)
	list(APPEND DPSIM_CXX_FLAGS ${OpenMP_CXX_FLAGS})
	list(APPEND DPSIM_LIBRARIES ${OpenMP_CXX_FLAGS})
endif()

add_library(dpsim ${DPSIM_SOURCES})
target_link_libraries(dpsim PUBLIC ${DPSIM_LIBRARIES})
target_include_directories(dpsim PUBLIC ${DPSIM_INCLUDE_DIRS})
target_compile_options(dpsim PUBLIC ${DPSIM_CXX_FLAGS})
