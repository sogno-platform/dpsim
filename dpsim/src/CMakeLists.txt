set(DPSIM_SOURCES
	Simulation.cpp
	MNASolver.cpp
	MNASolverDirect.cpp
	MNAStateSpaceContributor.cpp
	MNAStateSpaceExtractor.cpp
	DenseLUAdapter.cpp
	DirectLinearSolverConfiguration.cpp
	PFSolver.cpp
	PFSolverPowerPolar.cpp
	PFSolverPowerPolarSparse.cpp
	Utils.cpp
	Timer.cpp
	Event.cpp
	DataLogger.cpp
	Scheduler.cpp
	SequentialScheduler.cpp
	StateSpaceModalAnalysis.cpp
	ThreadScheduler.cpp
	ThreadLevelScheduler.cpp
	ThreadListScheduler.cpp
	DiakopticsSolver.cpp
	Interface.cpp
	InterfaceQueued.cpp
)

list(APPEND DPSIM_LIBRARIES
	dpsim-models
	filesystem
	readerwriterqueue::readerwriterqueue
)

if(WITH_MNASOLVERPLUGIN)
	list(APPEND DPSIM_LIBRARIES ${CMAKE_DL_LIBS})
	list(APPEND DPSIM_SOURCES MNASolverPlugin.cpp)
endif()

if(NOT MSVC)
	list(APPEND DPSIM_SOURCES RealTimeDataLogger.cpp)
endif()

if(WITH_RT)
	list(APPEND DPSIM_SOURCES RealTimeSimulation.cpp)
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

if(WITH_SPARSE)
	list(APPEND DPSIM_SOURCES SparseLUAdapter.cpp)
endif()

if(WITH_KLU)
	list(APPEND DPSIM_LIBRARIES SuiteSparse::KLU)
	list(APPEND DPSIM_SOURCES KLUAdapter.cpp)
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
		list(APPEND DPSIM_SOURCES GpuMagmaAdapter.cpp)
		list(APPEND DPSIM_LIBRARIES ${MAGMA_LIBRARIES})
		list(APPEND DPSIM_LIBRARIES ${CUDA_cusparse_LIBRARY})
		list(APPEND DPSIM_INCLUDE_DIRS ${MAGMA_INCLUDE_DIR})
	endif()
	if(WITH_CUDA_SPARSE)
		list(APPEND DPSIM_SOURCES GpuSparseAdapter.cpp)
		list(APPEND DPSIM_LIBRARIES ${CUDA_cusparse_LIBRARY})
	endif()
endif()

if(WITH_METAL)
	list(APPEND DPSIM_SOURCES GPU/Apple/MetalDenseAdapter.mm GPU/Apple/MetalSparseLUAdapter.mm GPU/Apple/MetalSparseBiCGSTABAdapter.mm GPU/Apple/MetalLinearSSNGroupAdapter.mm GPU/Apple/MetalVBRGeneratorAdapter.mm GPU/Apple/MetalResidentScheduler.mm)
	set_source_files_properties(GPU/Apple/MetalDenseAdapter.mm GPU/Apple/MetalSparseLUAdapter.mm GPU/Apple/MetalSparseBiCGSTABAdapter.mm GPU/Apple/MetalLinearSSNGroupAdapter.mm GPU/Apple/MetalVBRGeneratorAdapter.mm GPU/Apple/MetalResidentScheduler.mm PROPERTIES COMPILE_OPTIONS "-fno-objc-arc")
	list(APPEND DPSIM_LIBRARIES
		"-framework Foundation"
		"-framework Metal"
		"-framework MetalPerformanceShaders"
	)
endif()

if(WITH_HIP)
	if(NOT ROCM_ROOT)
		if(DEFINED ENV{ROCM_HOME})
			set(ROCM_ROOT "$ENV{ROCM_HOME}")
		elseif(DEFINED ENV{ROCM_PATH})
			set(ROCM_ROOT "$ENV{ROCM_PATH}")
		else()
			message(FATAL_ERROR "WITH_HIP requires ROCM_ROOT or ROCM_HOME to point to the ROCm SDK")
		endif()
	endif()
	find_path(HIP_INCLUDE_DIR hip/hip_runtime_api.h HINTS "${ROCM_ROOT}/include" REQUIRED)
	find_path(HIPSOLVER_INCLUDE_DIR hipsolver/hipsolver.h HINTS "${ROCM_ROOT}/include" REQUIRED)
	find_library(HIP_RUNTIME_LIBRARY NAMES amdhip64 HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
	find_library(HIPSOLVER_LIBRARY NAMES hipsolver HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
	find_library(HIPBLAS_LIBRARY NAMES hipblas HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
	find_library(ROCSPARSE_LIBRARY NAMES rocsparse HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
	find_library(HIPSPARSE_LIBRARY NAMES hipsparse HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
	list(APPEND DPSIM_SOURCES HipDenseAdapter.cpp HipSparseBiCGStabAdapter.cpp HipSparseQrAdapter.cpp HipLinearSSNFusedAdapter.cpp HipSG4BatchAdapter.cpp HipResidentKernels.hip HipLinearSSNFusedKernels.hip HipSG4BatchKernels.hip)
	list(APPEND DPSIM_INCLUDE_DIRS "${HIP_INCLUDE_DIR}" "${HIPSOLVER_INCLUDE_DIR}")
	list(APPEND DPSIM_LIBRARIES "${HIP_RUNTIME_LIBRARY}" "${HIPSOLVER_LIBRARY}" "${HIPBLAS_LIBRARY}" "${ROCSPARSE_LIBRARY}" "${HIPSPARSE_LIBRARY}")
	if(WITH_KLU)
		find_path(ROCSOLVER_INCLUDE_DIR rocsolver/rocsolver.h HINTS "${ROCM_ROOT}/include" REQUIRED)
		find_library(ROCSOLVER_LIBRARY NAMES rocsolver HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
		find_library(ROCBLAS_LIBRARY NAMES rocblas HINTS "${ROCM_ROOT}/lib" "${ROCM_ROOT}/lib64" REQUIRED)
		list(APPEND DPSIM_SOURCES HipSparseLURefactorAdapter.cpp)
		list(APPEND DPSIM_INCLUDE_DIRS "${ROCSOLVER_INCLUDE_DIR}")
		list(APPEND DPSIM_LIBRARIES "${ROCSOLVER_LIBRARY}" "${ROCBLAS_LIBRARY}")
	endif()
	set(CMAKE_BUILD_RPATH "${ROCM_ROOT}/lib;${ROCM_ROOT}/lib64")
endif()

if(WITH_OPENMP)
	list(APPEND DPSIM_SOURCES OpenMPLevelScheduler.cpp)
	list(APPEND DPSIM_LIBRARIES OpenMP::OpenMP_CXX)
endif()

if(WITH_VILLAS)
	list(APPEND DPSIM_LIBRARIES
		${VILLASNODE_LIBRARIES}
		${VILLASCOMMON_LIBRARIES}
		pthread
	)
	if(VILLASFPGA_LIBRARIES)
		list(APPEND DPSIM_LIBRARIES ${VILLASFPGA_LIBRARIES})
	endif()
endif()

add_library(dpsim ${DPSIM_SOURCES})
target_link_libraries(dpsim PUBLIC ${DPSIM_LIBRARIES})
target_include_directories(dpsim PUBLIC ${DPSIM_INCLUDE_DIRS})
target_compile_options(dpsim PUBLIC ${DPSIM_CXX_FLAGS})
if(WITH_HIP)
	target_compile_definitions(dpsim PUBLIC __HIP_PLATFORM_AMD__)
endif()
