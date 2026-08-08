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
#include <dpsim/MNASolverDirect.h>
#ifdef WITH_SPARSE
#include <dpsim/SparseLUAdapter.h>
#endif
#ifdef WITH_KLU
#include <dpsim/KLUAdapter.h>
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
