/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#pragma once

#include <dpsim/DataLogger.h>
#include <dpsim/DenseLUAdapter.h>
#include <dpsim/DirectLinearSolverConfiguration.h>
#include <dpsim/MNASolver.h>
#ifdef WITH_HIP
#include <dpsim/HipDenseAdapter.h>
#include <dpsim/HipSparseBiCGStabAdapter.h>
#include <dpsim/HipSparseQrAdapter.h>
#ifdef WITH_KLU
#include <dpsim/HipSparseLURefactorAdapter.h>
#endif
#endif
#include <dpsim/MNASolverDirect.h>
#ifdef WITH_SPARSE
#include <dpsim/SparseLUAdapter.h>
#endif
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
  static const std::vector<DirectLinearSolverImpl> mSupportedSolverImpls(void) {
    static std::vector<DirectLinearSolverImpl> ret = {
#ifdef WITH_MNASOLVERPLUGIN
        DirectLinearSolverImpl::Plugin,
#endif //WITH_MNASOLVERPLUGIN
#ifdef WITH_CUDA
        DirectLinearSolverImpl::CUDADense,
#ifdef WITH_CUDA_SPARSE
#endif // WITH_CUDA_SPARSE
        DirectLinearSolverImpl::CUDASparse,
#ifdef WITH_MAGMA
        DirectLinearSolverImpl::CUDAMagma,
#endif // WITH_MAGMA
#endif // WITH_CUDA
#ifdef WITH_METAL
        DirectLinearSolverImpl::MetalDense,
        DirectLinearSolverImpl::MetalSparseBiCGSTAB,
#endif // WITH_METAL
#ifdef WITH_HIP
        DirectLinearSolverImpl::HIPDense,
        DirectLinearSolverImpl::HIPSparseBiCGStab,
        DirectLinearSolverImpl::HIPSparseQR,
#ifdef WITH_KLU
        DirectLinearSolverImpl::HIPSparseLURefactor,
#endif
#endif
        DirectLinearSolverImpl::DenseLU,
#ifdef WITH_SPARSE
        DirectLinearSolverImpl::SparseLU,
#endif // WITH_SPARSE
#ifdef WITH_KLU
        DirectLinearSolverImpl::KLU
#endif // WITH_KLU
    };
    return ret;
  }

  /// sovlerImpl: choose the most advanced solver implementation available by default
  template <typename VarType>
  static std::shared_ptr<MnaSolver<VarType>> factory(
      String name, CPS::Domain domain = CPS::Domain::DP,
      CPS::Logger::Level logLevel = CPS::Logger::Level::info,
      DirectLinearSolverImpl implementation = mSupportedSolverImpls().back(),
      String pluginName = "plugin.so") {
    //To avoid regression we use KLU in case of undefined implementation
    if (implementation == DirectLinearSolverImpl::Undef) {
#ifdef WITH_KLU
      implementation = DirectLinearSolverImpl::KLU;
#else
      implementation = mSupportedSolverImpls().back();
#endif
    }
    CPS::Logger::Log log = CPS::Logger::get(
        "MnaSolverFactory", CPS::Logger::Level::info, CPS::Logger::Level::info);

    switch (implementation) {
      /* TODO: have only one "solver" object of type MnaSolverDirect and only use setDirectLinearSolverImplementation in the switch-case.
     * This is not done now, since MnaSolverDirect and MnaSolver are distinct classes - and someone might add another subclass of MnaSolver
     * to the project (MnaSolverIterative?). It is planned to merge MnaSolverDirect and MnaSolver anyway, so this won't happen.
     */
#ifdef WITH_SPARSE
    case DirectLinearSolverImpl::SparseLU: {
      SPDLOG_LOGGER_INFO(log, "creating SparseLUAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> sparseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      sparseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::SparseLU);
      return sparseSolver;
    }
#endif // WITH_SPARSE
    case DirectLinearSolverImpl::DenseLU: {
      SPDLOG_LOGGER_INFO(log, "creating DenseLUAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> denseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      denseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::DenseLU);
      return denseSolver;
    }
#ifdef WITH_KLU
    case DirectLinearSolverImpl::KLU: {
      SPDLOG_LOGGER_INFO(log, "creating KLUAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> kluSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      kluSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::KLU);
      return kluSolver;
    }
#endif
#ifdef WITH_CUDA
    case DirectLinearSolverImpl::CUDADense: {
      SPDLOG_LOGGER_INFO(log, "creating GpuDenseAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> gpuDenseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      gpuDenseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::CUDADense);
      return gpuDenseSolver;
    }
#ifdef WITH_CUDA_SPARSE
    case DirectLinearSolverImpl::CUDASparse: {
      SPDLOG_LOGGER_INFO(log,
                         "creating GpuSparseAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> gpuSparseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      gpuSparseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::CUDASparse);
      return gpuSparseSolver;
    }
#endif
#ifdef WITH_MAGMA
    case DirectLinearSolverImpl::CUDAMagma: {
      SPDLOG_LOGGER_INFO(log, "creating GpuMagmaAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> gpuMagmaSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      gpuMagmaSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::CUDAMagma);
      return gpuMagmaSolver;
    }
#endif
#endif
#ifdef WITH_METAL
    case DirectLinearSolverImpl::MetalDense: {
      SPDLOG_LOGGER_INFO(log,
                         "creating MetalDenseAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> metalSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      metalSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::MetalDense);
      return metalSolver;
    }
    case DirectLinearSolverImpl::MetalSparseBiCGSTAB: {
      SPDLOG_LOGGER_INFO(
          log, "creating MetalSparseBiCGSTABAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> metalSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      metalSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::MetalSparseBiCGSTAB);
      return metalSolver;
    }
#endif
#ifdef WITH_HIP
    case DirectLinearSolverImpl::HIPSparseQR: {
      SPDLOG_LOGGER_INFO(log,
                         "creating HipSparseQrAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> hipSparseQrSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      hipSparseQrSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::HIPSparseQR);
      return hipSparseQrSolver;
    }
    case DirectLinearSolverImpl::HIPSparseBiCGStab: {
      SPDLOG_LOGGER_INFO(
          log, "creating HipSparseBiCGStabAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> hipSparseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      hipSparseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::HIPSparseBiCGStab);
      return hipSparseSolver;
    }
    case DirectLinearSolverImpl::HIPSparseLURefactor: {
#ifdef WITH_KLU
      SPDLOG_LOGGER_INFO(
          log, "creating HipSparseLURefactorAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> hipSparseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      hipSparseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::HIPSparseLURefactor);
      return hipSparseSolver;
#else
      throw CPS::SystemError(
          "HIP sparse LU refactorization requires the KLU seed solver");
#endif
    }
    case DirectLinearSolverImpl::HIPDense: {
      SPDLOG_LOGGER_INFO(log, "creating HipDenseAdapter solver implementation");
      std::shared_ptr<MnaSolverDirect<VarType>> hipDenseSolver =
          std::make_shared<MnaSolverDirect<VarType>>(name, domain, logLevel);
      hipDenseSolver->setDirectLinearSolverImplementation(
          DirectLinearSolverImpl::HIPDense);
      return hipDenseSolver;
    }
#endif
#ifdef WITH_MNASOLVERPLUGIN
    case DirectLinearSolverImpl::Plugin:
      SPDLOG_LOGGER_INFO(log, "creating Plugin solver implementation");
      return std::make_shared<MnaSolverPlugin<VarType>>(pluginName, name,
                                                        domain, logLevel);
#endif
    default:
      throw CPS::SystemError("unsupported MNA implementation.");
    }
  }
};
} // namespace DPsim
