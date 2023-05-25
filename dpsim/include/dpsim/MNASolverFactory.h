/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#pragma once

#include <dpsim/MNASolver.h>
#include <dpsim/DataLogger.h>
#include <dpsim/MNASolverDirect.h>
#include <dpsim/DirectLinearSolverConfiguration.h>
#include <dpsim/DenseLUAdapter.h>
#include <dpsim/SparseLUAdapter.h>
#ifdef WITH_KLU
#include <dpsim/KLUAdapter.h>
#endif
#ifdef WITH_CUDA
#include <dpsim/GpuDenseAdapter.h>
#ifdef WITH_CUDA_SPARSE
#include <dpsim/GpuSparseAdapter.h>
#endif
#ifdef WITH_MAGMA
#include <dpsim/GpuMagmaAdapter.h>
#endif
#endif
#ifdef WITH_MNASOLVERPLUGIN
#include <dpsim/MNASolverPlugin.h>
#endif

namespace DPsim {

class MnaSolverFactory {
	public:

	/// MNA implementations supported by this compilation
	static const std::vector<CPS::DirectLinearSolverImpl> mSupportedSolverImpls(void) {
		static std::vector<CPS::DirectLinearSolverImpl> ret = {
#ifdef WITH_MNASOLVERPLUGIN
			CPS::DirectLinearSolverImpl::Plugin,
#endif //WITH_MNASOLVERPLUGIN
#ifdef WITH_CUDA
			CPS::DirectLinearSolverImpl::CUDADense,
	#ifdef WITH_CUDA_SPARSE
	#endif // WITH_CUDA_SPARSE
			CPS::DirectLinearSolverImpl::CUDASparse,
	#ifdef WITH_MAGMA
			CPS::DirectLinearSolverImpl::CUDAMagma,
	#endif // WITH_MAGMA
#endif // WITH_CUDA
			CPS::DirectLinearSolverImpl::DenseLU,
			CPS::DirectLinearSolverImpl::SparseLU,
#ifdef WITH_KLU
			CPS::DirectLinearSolverImpl::KLU
#endif //WITH_KLU
		};
		return ret;
	}

	/// sovlerImpl: choose the most advanced solver implementation available by default
	template <typename VarType>
	static std::shared_ptr<MnaSolver<VarType>> factory(String name,
		CPS::Domain domain = CPS::Domain::DP,
		std::shared_ptr<SolverParametersMNA> solverParams = SolverParametersMNA(),
		CPS::Logger::Level logLevel = CPS::Logger::Level::info,
		String pluginName = "plugin.so")
	{
		//To avoid regression we use SparseLU in case of undefined implementation
		if (solverParams->mDirectImpl == CPS::DirectLinearSolverImpl::Undef)
			solverParams->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::SparseLU);

		//
		CPS::Logger::Log log = CPS::Logger::get("MnaSolverFactory", CPS::Logger::Level::info, CPS::Logger::Level::info);

		switch(solverParams->mDirectImpl) {
		/* TODO: have only one "solver" object of type MnaSolverDirect and only use setDirectLinearSolverImplementation in the switch-case.
		 * This is not done now, since MnaSolverDirect and MnaSolver are distinct classes - and someone might add another subclass of MnaSolver
		 * to the project (MnaSolverIterative?). It is planned to merge MnaSolverDirect and MnaSolver anyway, so this won't happen. */
		case CPS::DirectLinearSolverImpl::SparseLU:
		{
			log->info("creating SparseLUAdapter solver implementation");
			std::shared_ptr<MnaSolverDirect<VarType>> sparseSolver = std::make_shared<MnaSolverDirect<VarType>>(name, domain, solverParams, logLevel);
			sparseSolver->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::SparseLU);
			return sparseSolver;
		}
		case CPS::DirectLinearSolverImpl::DenseLU:
		{
			log->info("creating DenseLUAdapter solver implementation");
			std::shared_ptr<MnaSolverDirect<VarType>> denseSolver = std::make_shared<MnaSolverDirect<VarType>>(name, domain, solverParams, logLevel);
			denseSolver->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::DenseLU);
			return denseSolver;
		}
#ifdef WITH_KLU
		case CPS::DirectLinearSolverImpl::KLU:
		{
			log->info("creating KLUAdapter solver implementation");
			std::shared_ptr<MnaSolverDirect<VarType>> kluSolver = std::make_shared<MnaSolverDirect<VarType>>(name, domain, solverParams, logLevel);
			kluSolver->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::KLU);
			return kluSolver;
		}
#endif
#ifdef WITH_CUDA
		case CPS::DirectLinearSolverImpl::CUDADense:
		{
			log->info("creating GpuDenseAdapter solver implementation");
			std::shared_ptr<MnaSolverDirect<VarType>> gpuDenseSolver = std::make_shared<MnaSolverDirect<VarType>>(name, domain, solverParams, logLevel);
			gpuDenseSolver->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::CUDADense);
			return gpuDenseSolver;
		}
#ifdef WITH_CUDA_SPARSE
		case CPS::DirectLinearSolverImpl::CUDASparse:
		{
			log->info("creating GpuSparseAdapter solver implementation");
			std::shared_ptr<MnaSolverDirect<VarType>> gpuSparseSolver = std::make_shared<MnaSolverDirect<VarType>>(name, domain, solverParams, logLevel);
			gpuSparseSolver->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::CUDASparse);
			return gpuSparseSolver;
		}
#endif
#ifdef WITH_MAGMA
		case CPS::DirectLinearSolverImpl::CUDAMagma:
		{
			log->info("creating GpuMagmaAdapter solver implementation");
			std::shared_ptr<MnaSolverDirect<VarType>> gpuMagmaSolver = std::make_shared<MnaSolverDirect<VarType>>(name, domain, solverParams, logLevel);
			gpuMagmaSolver->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::CUDAMagma);
			return gpuMagmaSolver;
		}
#endif
#endif
#ifdef WITH_MNASOLVERPLUGIN
		case CPS::DirectLinearSolverImpl::Plugin:
			log->info("creating Plugin solver implementation");
			return std::make_shared<MnaSolverPlugin<VarType>>(pluginName, name, domain, solverParams, logLevel);
#endif
		default:
			throw CPS::SystemError("unsupported MNA implementation.");

		}
	}
};
}
