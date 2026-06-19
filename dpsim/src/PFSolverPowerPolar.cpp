/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/PFSolverPowerPolar.h>

using namespace DPsim;
using namespace CPS;

PFSolverPowerPolar::PFSolverPowerPolar(CPS::String name,
                                       const CPS::SystemTopology &system,
                                       CPS::Real timeStep,
                                       CPS::Logger::Level logLevel)
    : PFSolver(name, system, timeStep, logLevel) {}

// void PFSolverPowerPolar::generateInitialSolution(Real time,
//                                                  bool keep_last_solution) {
//   resize_sol(mSystem.mNodes.size());
//   resize_complex_sol(mSystem.mNodes.size());
//   // update all components for the new time
//   for (auto comp : mSystem.mComponents) {
//     if (std::shared_ptr<CPS::SP::Ph1::Load> load =
//             std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//       if (load->use_profile) {
//         load->updatePQ(time);
//       }
//       load->calculatePerUnitParameters(mBaseApparentPower,
//                                        mSystem.mSystemOmega);
//       // load->calculatePerUnitParametersUpdateZ(mBaseApparentPower,
//       //                                  mSystem.mSystemOmega);
//     }
//     if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
//             std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
//       gen->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
//     }
//   }
//   // set initial solution for the new time for PQ busses
//   for (auto pq : mPQBuses) {
//     if (keep_last_solution == false) {
//       sol_V(pq->matrixNodeIndex()) = 1.0;
//       sol_D(pq->matrixNodeIndex()) = 0.0;
//       sol_P(pq->matrixNodeIndex()) = 0.0; // reset load injection to zero before adding load contribution, to avoid double counting in case of load with time-varying profile
//       sol_Q(pq->matrixNodeIndex()) = 0.0; // reset load injection to zero before adding load contribution, to avoid double counting in case of load with time-varying profile
//       // sol_V_complex(pq->matrixNodeIndex()) = Math::polar(
//       //   sol_V[pq->matrixNodeIndex()], sol_D[pq->matrixNodeIndex()]);
//     }
//     if (keep_last_solution == true) {
//       sol_P(pq->matrixNodeIndex()) = 0.0; // reset load injection to zero before adding load contribution, to avoid double counting in case of load with time-varying profile
//       sol_Q(pq->matrixNodeIndex()) = 0.0; // reset load injection to zero before adding load contribution, to avoid double counting in case of load with time-varying profile
//     }
//     for (auto comp : mSystem.mComponentsAtNode[pq]) {
//       if (std::shared_ptr<CPS::SP::Ph1::Load> load =
//               std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//         sol_P(pq->matrixNodeIndex()) -=
//             load->attributeTyped<CPS::Real>("P_pu")->get();
//         sol_Q(pq->matrixNodeIndex()) -=
//             load->attributeTyped<CPS::Real>("Q_pu")->get();
//       } else if (std::shared_ptr<CPS::SP::Ph1::SolidStateTransformer> sst =
//                      std::dynamic_pointer_cast<
//                          CPS::SP::Ph1::SolidStateTransformer>(comp)) {
//         sol_P(pq->matrixNodeIndex()) -= sst->getNodalInjection(pq).real();
//         sol_Q(pq->matrixNodeIndex()) -= sst->getNodalInjection(pq).imag();
//       } else if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi =
//                      std::dynamic_pointer_cast<
//                          CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
//         // TODO: add per-unit attributes to VSI and use here
//         sol_P(pq->matrixNodeIndex()) +=
//             vsi->attributeTyped<CPS::Real>("P_ref")->get() / mBaseApparentPower;
//         sol_Q(pq->matrixNodeIndex()) +=
//             vsi->attributeTyped<CPS::Real>("Q_ref")->get() / mBaseApparentPower;
//       } else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
//                      std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
//                          comp)) {
//         sol_P(pq->matrixNodeIndex()) +=
//             gen->attributeTyped<CPS::Real>("P_set_pu")->get();
//         sol_Q(pq->matrixNodeIndex()) +=
//             gen->attributeTyped<CPS::Real>("Q_set_pu")->get();
//       }
//     }
//         sol_S_complex(pq->matrixNodeIndex()) = CPS::Complex(
//         sol_P[pq->matrixNodeIndex()], sol_Q[pq->matrixNodeIndex()]);
//         sol_V_complex(pq->matrixNodeIndex()) = Math::polar(
//         sol_V[pq->matrixNodeIndex()], sol_D[pq->matrixNodeIndex()]);
//   }
//   for (auto pv : mPVBuses) {
//     if (keep_last_solution == false) {
//       sol_Q(pv->matrixNodeIndex()) = 0;
//       sol_D(pv->matrixNodeIndex()) = 0;
//       sol_P(pv->matrixNodeIndex()) = 0.0; // reset load injection to zero before adding load contribution, to avoid double counting in case of load with time-varying profile
//       sol_V(pv->matrixNodeIndex()) = 1.0; // LAST EDITED reset voltage to 1.0 pu before adding generator contribution, to avoid double counting in case of generator with time-varying set-point
//     }
//     if (keep_last_solution == true) {
//       //sol_Q(pv->matrixNodeIndex()) = 0;
//       sol_P(pv->matrixNodeIndex()) = 0.0; // reset load injection to zero before adding load contribution, to avoid double counting in case of load with time-varying profile
//     }
//     for (auto comp : mSystem.mComponentsAtNode[pv]) {
//       if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
//               std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
//                   comp)) {
//         sol_P(pv->matrixNodeIndex()) +=
//             gen->attributeTyped<CPS::Real>("P_set_pu")->get();
//         sol_V(pv->matrixNodeIndex()) =
//             gen->attributeTyped<CPS::Real>("V_set_pu")->get();
//       } else if (std::shared_ptr<CPS::SP::Ph1::Load> load =
//                      std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//         sol_P(pv->matrixNodeIndex()) -=
//             load->attributeTyped<CPS::Real>("P_pu")->get();
//         sol_Q(pv->matrixNodeIndex()) -=
//             load->attributeTyped<CPS::Real>("Q_pu")->get();
//       } else if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi =
//                      std::dynamic_pointer_cast<
//                          CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
//         sol_P(pv->matrixNodeIndex()) +=
//             vsi->attributeTyped<CPS::Real>("P_ref")->get() / mBaseApparentPower;
//       } else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
//                      std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(
//                          comp)) {
//         sol_P(pv->matrixNodeIndex()) +=
//             extnet->attributeTyped<CPS::Real>("p_inj")->get() /
//             mBaseApparentPower;
//         sol_V(pv->matrixNodeIndex()) =
//             extnet->attributeTyped<CPS::Real>("V_set_pu")->get();
//      }
//     }
//       sol_S_complex(pv->matrixNodeIndex()) = CPS::Complex(
//           sol_P[pv->matrixNodeIndex()], sol_Q[pv->matrixNodeIndex()]);
//       sol_V_complex(pv->matrixNodeIndex()) = Math::polar(
//           sol_V[pv->matrixNodeIndex()], sol_D[pv->matrixNodeIndex()]);
//   }
//   for (auto vd : mVDBuses) {
//     sol_P(vd->matrixNodeIndex()) = 0.0;
//     sol_Q(vd->matrixNodeIndex()) = 0.0;
//     sol_V(vd->matrixNodeIndex()) = 1.0;
//     sol_D(vd->matrixNodeIndex()) = 0.0;
//     // if external injection at VD bus, reset the voltage to injection's voltage set-point
//     for (auto comp : mSystem.mComponentsAtNode[vd]) {
//       if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
//               std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
//         sol_V(vd->matrixNodeIndex()) =
//             extnet->attributeTyped<CPS::Real>("V_set_pu")->get();
//         // sol_P(vd->matrixNodeIndex()) +=
//         //     extnet->attributeTyped<CPS::Real>("p_inj")->get() /
//         //     mBaseApparentPower; // Todo add p_set q_set to extnet
//         // sol_Q(vd->matrixNodeIndex()) +=
//         //     extnet->attributeTyped<CPS::Real>("q_inj")->get() /
//         //     mBaseApparentPower;
//       }
//       // if load at VD bus, substract P and Q
//       else if (std::shared_ptr<CPS::SP::Ph1::Load> load =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//         sol_P(vd->matrixNodeIndex()) -=
//             load->attributeTyped<CPS::Real>("P_pu")->get();
//         sol_Q(vd->matrixNodeIndex()) -=
//             load->attributeTyped<CPS::Real>("Q_pu")->get();
//       }
//       // if generator at VD, add P_set Q_Set
//       else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
//                        comp)) {
//         sol_P(vd->matrixNodeIndex()) +=
//             gen->attributeTyped<CPS::Real>("P_set_pu")->get();
//         sol_Q(vd->matrixNodeIndex()) +=
//             gen->attributeTyped<CPS::Real>("Q_set_pu")->get();
//       }
//     }
//     // if generator at VD bus, reset the voltage to generator's set-point
//     if (!mSynchronGenerators.empty()) {
//       for (auto gen : mSynchronGenerators) {
//         if (gen->node(0)->matrixNodeIndex() == vd->matrixNodeIndex())
//           sol_V(vd->matrixNodeIndex()) =
//               gen->attributeTyped<CPS::Real>("V_set_pu")->get();
//       }
//     }
//     sol_S_complex(vd->matrixNodeIndex()) = CPS::Complex(
//         sol_P[vd->matrixNodeIndex()], sol_Q[vd->matrixNodeIndex()]);
//     sol_V_complex(vd->matrixNodeIndex()) = Math::polar(
//         sol_V[vd->matrixNodeIndex()], sol_D[vd->matrixNodeIndex()]);
//   }
//   solutionInitialized = true;
//   solutionComplexInitialized = true;

//   Pesp = sol_P;
//   Qesp = sol_Q;

//   //GIVING INITIAL SOLUTION

//   SPDLOG_LOGGER_INFO(mSLog, "#### Initial solution: ");
//   SPDLOG_LOGGER_INFO(mSLog, "Name\tP\t\tQ\t\tV\t\tD");
//   for (auto node : mSystem.mNodes) {
//     SPDLOG_LOGGER_INFO(mSLog, "{}\t{}\t{}\t{}\t{}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->name(),
//     sol_P[node->matrixNodeIndex()], sol_Q[node->matrixNodeIndex()],
//     sol_V[node->matrixNodeIndex()], sol_D[node->matrixNodeIndex()]);
//   }

//   // SPDLOG_LOGGER_INFO(mSLog, "#### Initial solution: ");
//   // SPDLOG_LOGGER_INFO(mSLog, "P\t\tQ\t\tV\t\tD");
//   // for (UInt i = 0; i < mSystem.mNodes.size(); ++i) {
//   //   SPDLOG_LOGGER_INFO(mSLog, "{}\t{}\t{}\t{}", sol_P[i], sol_Q[i], sol_V[i],
//   //                      sol_D[i]);
//   // }
//   mSLog->flush();
// }

// void PFSolverPowerPolar::generateInitialSolution(Real time,
//                                                  bool keep_last_solution) {
//   const UInt n = mSystem.mNodes.size();

//   // Preserve previous converged voltage state before resizing/clearing
//   CPS::Vector old_V = sol_V;
//   CPS::Vector old_D = sol_D;
//   bool can_keep =
//       keep_last_solution &&
//       solutionInitialized &&
//       sol_V.size() == n &&
//       sol_D.size() == n;

//   resize_sol(n);
//   resize_complex_sol(n);

//   // Restore previous voltage guess if available
//   if (can_keep) {
//     sol_V = old_V;
//     sol_D = old_D;
//   }

//   // update all components for the new time
//   for (auto comp : mSystem.mComponents) {
//     if (auto load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//       if (load->use_profile) {
//         load->updatePQ(time);
//       }
//       load->calculatePerUnitParameters(mBaseApparentPower,
//                                        mSystem.mSystemOmega);
//     }

//     if (auto gen =
//             std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
//       gen->calculatePerUnitParameters(mBaseApparentPower,
//                                       mSystem.mSystemOmega);
//     }
//   }

//   // ---------- PQ buses ----------
//   for (auto pq : mPQBuses) {
//     UInt idx = pq->matrixNodeIndex();

//     sol_P(idx) = 0.0;
//     sol_Q(idx) = 0.0;

//     if (!can_keep) {
//       sol_V(idx) = 1.0;
//       sol_D(idx) = 0.0;
//     }

//     for (auto comp : mSystem.mComponentsAtNode[pq]) {
//       if (auto load =
//               std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//         sol_P(idx) -= load->attributeTyped<CPS::Real>("P_pu")->get();
//         sol_Q(idx) -= load->attributeTyped<CPS::Real>("Q_pu")->get();
//       }
//       else if (auto sst =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::SolidStateTransformer>(comp)) {
//         sol_P(idx) -= sst->getNodalInjection(pq).real();
//         sol_Q(idx) -= sst->getNodalInjection(pq).imag();
//       }
//       else if (auto vsi =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
//         sol_P(idx) += vsi->attributeTyped<CPS::Real>("P_ref")->get()
//                       / mBaseApparentPower;
//         sol_Q(idx) += vsi->attributeTyped<CPS::Real>("Q_ref")->get()
//                       / mBaseApparentPower;
//       }
//       else if (auto gen =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
//         sol_P(idx) += gen->attributeTyped<CPS::Real>("P_set_pu")->get();
//         sol_Q(idx) += gen->attributeTyped<CPS::Real>("Q_set_pu")->get();
//       }
//     }

//     sol_S_complex(idx) = CPS::Complex(sol_P(idx), sol_Q(idx));
//     sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
//   }

//   // ---------- PV buses ----------
//   for (auto pv : mPVBuses) {
//     UInt idx = pv->matrixNodeIndex();

//     sol_P(idx) = 0.0;
//     sol_Q(idx) = 0.0;

//     if (!can_keep) {
//       sol_D(idx) = 0.0;
//       sol_V(idx) = 1.0;
//     }

//     for (auto comp : mSystem.mComponentsAtNode[pv]) {
//       if (auto gen =
//               std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
//         sol_P(idx) += gen->attributeTyped<CPS::Real>("P_set_pu")->get();

//         // PV bus voltage magnitude is fixed by setpoint
//         sol_V(idx) = gen->attributeTyped<CPS::Real>("V_set_pu")->get();
//       }
//       else if (auto load =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//         sol_P(idx) -= load->attributeTyped<CPS::Real>("P_pu")->get();
//         sol_Q(idx) -= load->attributeTyped<CPS::Real>("Q_pu")->get();
//       }
//       else if (auto vsi =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
//         sol_P(idx) += vsi->attributeTyped<CPS::Real>("P_ref")->get()
//                       / mBaseApparentPower;
//       }
//       else if (auto extnet =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
//         // For PV network injection, use P if this is really intended as PV.
//         sol_P(idx) += extnet->attributeTyped<CPS::Real>("p_inj")->get()
//                       / mBaseApparentPower;
//         sol_V(idx) = extnet->attributeTyped<CPS::Real>("V_set_pu")->get();
//       }
//     }

//     sol_S_complex(idx) = CPS::Complex(sol_P(idx), sol_Q(idx));
//     sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
//   }

//   // ---------- VD / slack buses ----------
//   for (auto vd : mVDBuses) {
//     UInt idx = vd->matrixNodeIndex();

//     sol_P(idx) = 0.0;
//     sol_Q(idx) = 0.0;

//     // Slack angle must remain reference
//     sol_D(idx) = 0.0;
//     sol_V(idx) = 1.0;

//     for (auto comp : mSystem.mComponentsAtNode[vd]) {
//       if (auto extnet =
//               std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
//         sol_V(idx) = extnet->attributeTyped<CPS::Real>("V_set_pu")->get();

//         // IMPORTANT:
//         // Do NOT initialize slack P/Q from old p_inj/q_inj.
//         // Slack P/Q are outputs after convergence.
//       }
//       else if (auto load =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
//         sol_P(idx) -= load->attributeTyped<CPS::Real>("P_pu")->get();
//         sol_Q(idx) -= load->attributeTyped<CPS::Real>("Q_pu")->get();
//       }
//       else if (auto gen =
//                    std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
//         sol_P(idx) += gen->attributeTyped<CPS::Real>("P_set_pu")->get();
//         sol_Q(idx) += gen->attributeTyped<CPS::Real>("Q_set_pu")->get();
//         sol_V(idx) = gen->attributeTyped<CPS::Real>("V_set_pu")->get();
//       }
//     }

//     sol_S_complex(idx) = CPS::Complex(sol_P(idx), sol_Q(idx));
//     sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
//   }

//   solutionInitialized = true;
//   solutionComplexInitialized = true;

//   Pesp = sol_P;
//   Qesp = sol_Q;
// }

void PFSolverPowerPolar::generateInitialSolution(Real time,
                                                 bool keep_last_solution) {
  const UInt n = mSystem.mNodes.size();

  bool can_keep = keep_last_solution && mHasLastConvergedSolution &&
                  mLastConvergedV.size() == n && mLastConvergedD.size() == n;

  SPDLOG_LOGGER_INFO(mSLog,
                     "PF initialization: keep_last_solution={}, can_keep={}",
                     keep_last_solution, can_keep);

  resize_sol(n);
  resize_complex_sol(n);

  if (can_keep) {
    sol_V = mLastConvergedV;
    sol_D = mLastConvergedD;
  }

  // update components
  for (auto comp : mSystem.mComponents) {
    if (auto load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
      if (load->use_profile)
        load->updatePQ(time);

      load->calculatePerUnitParameters(mBaseApparentPower,
                                       mSystem.mSystemOmega);
    }

    if (auto gen =
            std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
      gen->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
    }
  }

  // PQ buses
  for (auto pq : mPQBuses) {
    UInt idx = pq->matrixNodeIndex();

    sol_P(idx) = 0.0;
    sol_Q(idx) = 0.0;

    if (!can_keep) {
      sol_V(idx) = 1.0;
      sol_D(idx) = 0.0;
    }

    for (auto comp : mSystem.mComponentsAtNode[pq]) {
      if (auto load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
        sol_P(idx) -= load->attributeTyped<CPS::Real>("P_pu")->get();
        sol_Q(idx) -= load->attributeTyped<CPS::Real>("Q_pu")->get();
      } else if (auto sst = std::dynamic_pointer_cast<
                     CPS::SP::Ph1::SolidStateTransformer>(comp)) {
        sol_P(idx) -= sst->getNodalInjection(pq).real();
        sol_Q(idx) -= sst->getNodalInjection(pq).imag();
      } else if (auto vsi = std::dynamic_pointer_cast<
                     CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
        sol_P(idx) +=
            vsi->attributeTyped<CPS::Real>("P_ref")->get() / mBaseApparentPower;
        sol_Q(idx) +=
            vsi->attributeTyped<CPS::Real>("Q_ref")->get() / mBaseApparentPower;
      } else if (auto gen =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                         comp)) {
        sol_P(idx) += gen->attributeTyped<CPS::Real>("P_set_pu")->get();
        sol_Q(idx) += gen->attributeTyped<CPS::Real>("Q_set_pu")->get();
      }
    }

    sol_S_complex(idx) = CPS::Complex(sol_P(idx), sol_Q(idx));
    sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
  }

  // PV buses
  for (auto pv : mPVBuses) {
    UInt idx = pv->matrixNodeIndex();

    sol_P(idx) = 0.0;
    sol_Q(idx) = 0.0;

    if (!can_keep) {
      sol_D(idx) = 0.0;
      sol_V(idx) = 1.0;
    }

    for (auto comp : mSystem.mComponentsAtNode[pv]) {
      if (auto gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
              comp)) {
        sol_P(idx) += gen->attributeTyped<CPS::Real>("P_set_pu")->get();
        sol_V(idx) = gen->attributeTyped<CPS::Real>("V_set_pu")->get();
      } else if (auto load =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
        sol_P(idx) -= load->attributeTyped<CPS::Real>("P_pu")->get();
        sol_Q(idx) -= load->attributeTyped<CPS::Real>("Q_pu")->get();
      } else if (auto vsi = std::dynamic_pointer_cast<
                     CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
        sol_P(idx) +=
            vsi->attributeTyped<CPS::Real>("P_ref")->get() / mBaseApparentPower;
      } else if (auto extnet =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(
                         comp)) {
        sol_P(idx) += extnet->attributeTyped<CPS::Real>("p_inj")->get() /
                      mBaseApparentPower;
        sol_V(idx) = extnet->attributeTyped<CPS::Real>("V_set_pu")->get();
      }
    }

    sol_S_complex(idx) = CPS::Complex(sol_P(idx), sol_Q(idx));
    sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
  }

  // VD / slack buses
  for (auto vd : mVDBuses) {
    UInt idx = vd->matrixNodeIndex();

    sol_P(idx) = 0.0;
    sol_Q(idx) = 0.0;
    sol_D(idx) = 0.0;
    sol_V(idx) = 1.0;

    for (auto comp : mSystem.mComponentsAtNode[vd]) {
      if (auto extnet =
              std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
        sol_V(idx) = extnet->attributeTyped<CPS::Real>("V_set_pu")->get();
      } else if (auto load =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
        sol_P(idx) -= load->attributeTyped<CPS::Real>("P_pu")->get();
        sol_Q(idx) -= load->attributeTyped<CPS::Real>("Q_pu")->get();
      } else if (auto gen =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                         comp)) {
        sol_P(idx) += gen->attributeTyped<CPS::Real>("P_set_pu")->get();
        sol_Q(idx) += gen->attributeTyped<CPS::Real>("Q_set_pu")->get();
        sol_V(idx) = gen->attributeTyped<CPS::Real>("V_set_pu")->get();
      }
    }

    sol_S_complex(idx) = CPS::Complex(sol_P(idx), sol_Q(idx));
    sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
  }

  solutionInitialized = true;
  solutionComplexInitialized = true;

  Pesp = sol_P;
  Qesp = sol_Q;
}

void PFSolverPowerPolar::calculateMismatch() {
  UInt npqpv = mNumPQBuses + mNumPVBuses;
  UInt k;
  mF.setZero();

  for (UInt a = 0; a < npqpv; ++a) {
    // For PQ and PV buses calculate active power mismatch
    k = mPQPVBusIndices[a];
    mF(a) = Pesp.coeff(k) - P(k);

    //only for PQ buses calculate reactive power mismatch
    if (a < mNumPQBuses)
      mF(a + npqpv) = Qesp.coeff(k) - Q(k);
  }
}

void PFSolverPowerPolar::calculateJacobian() {
  UInt npqpv = mNumPQBuses + mNumPVBuses;
  double val;
  UInt k, j;
  UInt da, db;

  mJ.setZero();

  //J1
  for (UInt a = 0; a < npqpv; ++a) { //rows
    k = mPQPVBusIndices[a];
    //diagonal
    mJ.coeffRef(a, a) = -Q(k) - B(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

    //non diagonal elements
    for (UInt b = 0; b < npqpv; ++b) {
      if (b != a) {
        j = mPQPVBusIndices[b];
        val = sol_V.coeff(k) * sol_V.coeff(j) *
              (G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)) -
               B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
        //if (val != 0.0)
        mJ.coeffRef(a, b) = val;
      }
    }
  }

  //J2
  da = 0;
  db = npqpv;
  for (UInt a = 0; a < npqpv; ++a) { //rows
    k = mPQPVBusIndices[a];
    //diagonal
    //std::cout << "J2D:" << (a + da) << "," << (a + db) << std::endl;
    if (a < mNumPQBuses)
      mJ.coeffRef(a + da, a + db) =
          P(k) + G(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

    //non diagonal elements
    for (UInt b = 0; b < mNumPQBuses; ++b) {
      if (b != a) {
        j = mPQPVBusIndices[b];
        val = sol_V.coeff(k) * sol_V.coeff(j) *
              (G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)) +
               B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
        //if (val != 0.0)
        //std::cout << "J2ij:" << (a + da) << "," << (b + db) << std::endl;
        mJ.coeffRef(a + da, b + db) = val;
      }
    }
  }

  //J3
  da = npqpv;
  db = 0;
  for (UInt a = 0; a < mNumPQBuses; ++a) { //rows
    k = mPQPVBusIndices[a];
    //diagonal
    //std::cout << "J3:" << (a + da) << "," << (a + db) << std::endl;
    mJ.coeffRef(a + da, a + db) =
        P(k) - G(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

    //non diagonal elements
    for (UInt b = 0; b < npqpv; ++b) {
      if (b != a) {
        j = mPQPVBusIndices[b];
        val = sol_V.coeff(k) * sol_V.coeff(j) *
              (G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)) +
               B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
        //if (val != 0.0)
        //std::cout << "J3:" << (a + da) << "," << (b + db) << std::endl;
        mJ.coeffRef(a + da, b + db) = -val;
      }
    }
  }

  //J4
  da = npqpv;
  db = npqpv;
  for (UInt a = 0; a < mNumPQBuses; ++a) { //rows
    k = mPQPVBusIndices[a];
    //diagonal
    //std::cout << "J4:" << (a + da) << "," << (a + db) << std::endl;
    mJ.coeffRef(a + da, a + db) =
        Q(k) - B(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

    //non diagonal elements
    for (UInt b = 0; b < mNumPQBuses; ++b) {
      if (b != a) {
        j = mPQPVBusIndices[b];
        val = sol_V.coeff(k) * sol_V.coeff(j) *
              (G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)) -
               B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
        if (val != 0.0) {
          //std::cout << "J4:" << (a + da) << "," << (b + db) << std::endl;
          mJ.coeffRef(a + da, b + db) = val;
        }
      }
    }
  }
}

// void PFSolverPowerPolar::updateSolution() {
//   UInt npqpv = mNumPQBuses + mNumPVBuses;
//   UInt k;

//   for (UInt a = 0; a < npqpv; ++a) {
//     k = mPQPVBusIndices[a];
//     sol_D(k) += mX.coeff(a);

//     if (a < mNumPQBuses)
//       sol_V(k) = sol_V.coeff(k) * (1.0 + mX.coeff(a + npqpv));
//   }

//   //Correction for PV buses
//   for (UInt i = mNumPQBuses; i < npqpv; ++i) {
//     k = mPQPVBusIndices[i];
//     Complex v = sol_Vcx(k);
//     // v *= Model.buses[k].v_set_point / abs(v);
//     sol_V(k) = abs(v);
//     sol_D(k) = arg(v);
//   }
// }

// void PFSolverPowerPolar::updateSolution() {
//   UInt npqpv = mNumPQBuses + mNumPVBuses;
//   UInt k;

//   for (UInt a = 0; a < npqpv; ++a) {
//     k = mPQPVBusIndices[a];

//     // angle update for PQ and PV buses
//     double dD = std::clamp(mX.coeff(a), -0.3, 0.3);
//     sol_D(k) += dD;
//     sol_D(k) = std::remainder(sol_D(k), 2.0 * M_PI);

//     // voltage update only for PQ buses
//     if (a < mNumPQBuses) {
//       double dVrel = std::clamp(mX.coeff(a + npqpv), -0.5, 0.5);

//       // multiplicative positive update
//       sol_V(k) *= std::exp(dVrel);

//       // optional safety limits
//       sol_V(k) = std::clamp(sol_V(k), 0.05, 2.0);
//     }
//   }

//   // rebuild complex voltage from polar variables
//   for (auto node : mSystem.mNodes) {
//     UInt idx = node->matrixNodeIndex();
//     sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
//   }
// }

void PFSolverPowerPolar::updateSolution() {
  UInt npqpv = mNumPQBuses + mNumPVBuses;

  // Scale the whole Newton step by one factor (=1 near solution) to bound the
  // max voltage/angle change without altering the search direction.
  const double maxDVpu = 0.1;      // max |dV| per step [pu]
  const double maxDThetaRad = 0.2; // max |dTheta| per step [rad]

  // mX: [0,npqpv) angle incr (PQ+PV), then rel. voltage dV/V (PQ only).
  double scale = 1.0;
  for (UInt a = 0; a < npqpv; ++a) {
    double dTheta = std::abs(mX.coeff(a));
    if (dTheta > maxDThetaRad)
      scale = std::min(scale, maxDThetaRad / dTheta);
  }
  for (UInt b = 0; b < mNumPQBuses; ++b) {
    double dVrel = std::abs(mX.coeff(npqpv + b));
    if (dVrel > maxDVpu)
      scale = std::min(scale, maxDVpu / dVrel);
  }

  for (UInt a = 0; a < npqpv; ++a) {
    UInt k = mPQPVBusIndices[a];
    sol_D(k) += scale * mX.coeff(a);
    // additive-relative update, consistent with the Jacobian
    if (a < mNumPQBuses)
      sol_V(k) *= (1.0 + scale * mX.coeff(a + npqpv));
  }

  for (auto node : mSystem.mNodes) {
    UInt idx = node->matrixNodeIndex();
    sol_V_complex(idx) = Math::polar(sol_V(idx), sol_D(idx));
  }
}

void PFSolverPowerPolar::setSolution() {

  if (!isConverged) {
    SPDLOG_LOGGER_WARN(mSLog, "Not converged within {} iterations",
                       mIterations);

    SPDLOG_LOGGER_WARN(mSLog, "Writing last iterate to result state "
                              "(not stored as warm-start solution).");
  } else {
    calculatePAndQAtSlackBus();
    calculateQAtPVBuses();
    calculatePAndQInjectionPQBuses();

    mLastConvergedV = sol_V;
    mLastConvergedD = sol_D;
    mHasLastConvergedSolution = true;

    SPDLOG_LOGGER_INFO(mSLog, "Converged in {} iterations", mIterations);
  }

  SPDLOG_LOGGER_INFO(mSLog, "Solution written to result state:");

  SPDLOG_LOGGER_INFO(mSLog, "Name\tP\t\tQ\t\tV\t\tD");

  for (auto node : mSystem.mNodes) {
    UInt idx = node->matrixNodeIndex();

    SPDLOG_LOGGER_INFO(
        mSLog, "{}\t{}\t{}\t{}\t{}",
        std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->name(),
        sol_P[idx], sol_Q[idx], sol_V[idx], sol_D[idx]);
  }

  mSLog->flush();

  for (UInt i = 0; i < mSystem.mNodes.size(); ++i) {
    sol_S_complex(i) = CPS::Complex(sol_P.coeff(i), sol_Q.coeff(i));
    sol_V_complex(i) = Math::polar(sol_V.coeff(i), sol_D.coeff(i));
  }

  for (auto node : mSystem.mNodes) {
    auto simNode = std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node);

    UInt idx = node->matrixNodeIndex();

    simNode->setVoltage(sol_V_complex(idx) * mBaseVoltageAtNode[node]);

    simNode->setPower(sol_S_complex(idx) * mBaseApparentPower);
  }

  calculateBranchFlow();
  calculateNodalInjection();
}

// void PFSolverPowerPolar::setSolution() {
//   if (!isConverged) {
//     SPDLOG_LOGGER_INFO(mSLog, "Not converged within {} iterations",
//                        mIterations);
//     SPDLOG_LOGGER_INFO(mSLog, "Last known Solution: ");

//     SPDLOG_LOGGER_INFO(mSLog, "Name\tP\t\tQ\t\tV\t\tD");
//     for (auto node : mSystem.mNodes) {
//       SPDLOG_LOGGER_INFO(mSLog, "{}\t{}\t{}\t{}\t{}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->name(),
//       sol_P[node->matrixNodeIndex()], sol_Q[node->matrixNodeIndex()],
//       sol_V[node->matrixNodeIndex()], sol_D[node->matrixNodeIndex()]);
//     }
//   } else {
//     calculatePAndQAtSlackBus();
//     calculateQAtPVBuses();
//     // calculatePAndQInjectionPQBuses();
//     SPDLOG_LOGGER_INFO(mSLog, "converged in {} iterations", mIterations);
//     SPDLOG_LOGGER_INFO(mSLog, "Solution: ");

//     SPDLOG_LOGGER_INFO(mSLog, "Name\tP\t\tQ\t\tV\t\tD");
//     for (auto node : mSystem.mNodes) {
//       SPDLOG_LOGGER_INFO(mSLog, "{}\t{}\t{}\t{}\t{}", std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->name(),
//       sol_P[node->matrixNodeIndex()], sol_Q[node->matrixNodeIndex()],
//       sol_V[node->matrixNodeIndex()], sol_D[node->matrixNodeIndex()]);
//     }
//   }
//   for (UInt i = 0; i < mSystem.mNodes.size(); ++i) {
//     sol_S_complex(i) = CPS::Complex(sol_P.coeff(i), sol_Q.coeff(i));
//     sol_V_complex(i) = CPS::Complex(sol_V.coeff(i) * cos(sol_D.coeff(i)),
//                                     sol_V.coeff(i) * sin(sol_D.coeff(i)));
//   }

//   // update voltage and power at each node
//   for (auto node : mSystem.mNodes) {
//     std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->setVoltage(
//         sol_V_complex(node->matrixNodeIndex()) * mBaseVoltageAtNode[node]);
//     std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->setPower(
//         sol_S_complex(node->matrixNodeIndex()) * mBaseApparentPower);
//   }
//   calculateBranchFlow();
//   calculateNodalInjection();
// }

void PFSolverPowerPolar::calculateBranchFlow() {
  for (auto line : mLines) {
    VectorComp v(2);
    v(0) = sol_V_complex.coeff(line->node(0)->matrixNodeIndex());
    v(1) = sol_V_complex.coeff(line->node(1)->matrixNodeIndex());
    /// I = Y * V
    VectorComp current = line->Y_element() * v;
    /// pf on branch [S_01; S_10] = [V_0 * conj(I_0); V_1 * conj(I_1)]
    VectorComp flow_on_branch = v.array() * current.conjugate().array();
    line->updateBranchFlow(current, flow_on_branch);
  }
  for (auto trafo : mTransformers) {
    VectorComp v(2);
    v(0) = sol_V_complex.coeff(trafo->node(0)->matrixNodeIndex());
    v(1) = sol_V_complex.coeff(trafo->node(1)->matrixNodeIndex());
    /// I = Y * V
    VectorComp current = trafo->Y_element() * v;
    /// pf on branch [S_01; S_10] = [V_0 * conj(I_0); V_1 * conj(I_1)]
    VectorComp flow_on_branch = v.array() * current.conjugate().array();
    trafo->updateBranchFlow(current, flow_on_branch);
  }
}

void PFSolverPowerPolar::calculateNodalInjection() {
  for (auto node : mSystem.mNodes) {
    std::list<std::shared_ptr<CPS::SP::Ph1::PiLine>> lines;
    for (auto comp : mSystem.mComponentsAtNode[node]) {
      if (std::shared_ptr<CPS::SP::Ph1::PiLine> line =
              std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp)) {
        line->storeNodalInjection(sol_S_complex.coeff(node->matrixNodeIndex()));
        lines.push_back(line);
        break;
      }
    }
    if (lines.empty()) {
      for (auto comp : mSystem.mComponentsAtNode[node]) {
        if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo =
                std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp)) {
          trafo->storeNodalInjection(
              sol_S_complex.coeff(node->matrixNodeIndex()));
          break;
        }
      }
    }
  }
}

Real PFSolverPowerPolar::P(UInt k) {
  Real val = 0.0;
  for (UInt j = 0; j < mSystem.mNodes.size(); ++j) {
    val += sol_V.coeff(j) * (G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)) +
                             B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
  }
  return sol_V.coeff(k) * val;
}

Real PFSolverPowerPolar::Q(UInt k) {
  Real val = 0.0;
  for (UInt j = 0; j < mSystem.mNodes.size(); ++j) {
    val += sol_V.coeff(j) * (G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)) -
                             B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
  }
  return sol_V.coeff(k) * val;
}

void PFSolverPowerPolar::calculatePAndQAtSlackBus() {
  for (auto topoNode : mVDBuses) {
    auto node_idx = topoNode->matrixNodeIndex();

    // Net nodal injection into the network: S_inj = V * conj(YV)
    CPS::Complex I(0.0, 0.0);
    for (UInt j = 0; j < mSystem.mNodes.size(); ++j)
      I += mY.coeff(node_idx, j) * sol_Vcx(j);

    CPS::Complex S = sol_Vcx(node_idx) * conj(I);

    // Generator/source power: S_gen = S_inj + S_load
    CPS::Complex Sgen = S;
    for (auto comp : mSystem.mComponentsAtNode[topoNode]) {
      if (auto loadPtr = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
        Sgen += CPS::Complex(**(loadPtr->mActivePowerPerUnit),
                             **(loadPtr->mReactivePowerPerUnit));
      }
    }

    // Update connected VD source/generator with actual generated power
    for (auto comp : mSystem.mComponentsAtNode[topoNode]) {
      if (auto extnetPtr =
              std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
        extnetPtr->updatePowerInjection(Sgen * mBaseApparentPower);
        break;
      }

      if (auto sgPtr =
              std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                  comp)) {
        sgPtr->updatePowerInjection(Sgen * mBaseApparentPower);
        break;
      }
    }

    // Store net nodal injection, not generator power
    sol_P(node_idx) = S.real();
    sol_Q(node_idx) = S.imag();
  }
}
void PFSolverPowerPolar::calculateQAtPVBuses() {
  for (auto topoNode : mPVBuses) {
    auto node_idx = topoNode->matrixNodeIndex();

    // Net nodal injection into the network: S_inj = V * conj(YV)
    CPS::Complex I(0.0, 0.0);
    for (UInt j = 0; j < mSystem.mNodes.size(); ++j)
      I += mY.coeff(node_idx, j) * sol_Vcx(j);

    CPS::Complex S = sol_Vcx(node_idx) * conj(I);

    // Generator power: S_gen = S_inj + S_load
    CPS::Complex Sgen = S;
    for (auto comp : mSystem.mComponentsAtNode[topoNode]) {
      if (auto loadPtr = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
        Sgen += CPS::Complex(**(loadPtr->mActivePowerPerUnit),
                             **(loadPtr->mReactivePowerPerUnit));
      }
    }

    // Update PV generator with actual generator Q
    for (auto comp : mSystem.mComponentsAtNode[topoNode]) {
      if (auto sgPtr =
              std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                  comp)) {
        sgPtr->updateReactivePowerInjection(Sgen * mBaseApparentPower);
        break;
      }
    }

    // Store net nodal Q injection, not generator Q
    sol_Q(node_idx) = S.imag();
  }
}

void PFSolverPowerPolar::calculatePAndQInjectionPQBuses() {
  // calculates apparent power injection at PQ buses flowing to other nodes (i.e. S_inj_to_other = S_inj - S_shunt, with S_inj = S_gen - S_load)
  for (auto topoNode : mPQBuses) {
    auto node_idx = topoNode->matrixNodeIndex();

    // calculate power flowing out of the node into the admittance matrix (i.e. S_inj)
    CPS::Complex I(0.0, 0.0);
    for (UInt j = 0; j < mSystem.mNodes.size(); ++j)
      I += mY.coeff(node_idx, j) * sol_Vcx(j);
    CPS::Complex S = sol_Vcx(node_idx) * conj(I);

    // Subtracting shunt power to obtain power injection flowing from this node to the other nodes (i.e. S_inj_to_other)
    CPS::Real V = sol_V.coeff(node_idx);
    for (auto comp : mSystem.mComponentsAtNode[topoNode])
      if (auto shuntPtr = std::dynamic_pointer_cast<CPS::SP::Ph1::Shunt>(comp))
        // capacitive susceptance is positive --> q is injected into the node
        S += std::pow(V, 2) * Complex(-**(shuntPtr->mConductancePerUnit),
                                      **(shuntPtr->mSusceptancePerUnit));

    // TODO: check whether S_inj_to_other should be stored in sol_P and sol_Q or rather S_inj
    sol_P(node_idx) = S.real();
    sol_Q(node_idx) = S.imag();
  }
}

void PFSolverPowerPolar::resize_sol(Int n) {
  sol_P = CPS::Vector(n);
  sol_Q = CPS::Vector(n);
  sol_V = CPS::Vector(n);
  sol_D = CPS::Vector(n);
  sol_P.setZero(n);
  sol_Q.setZero(n);
  sol_V.setZero(n);
  sol_D.setZero(n);
}

void PFSolverPowerPolar::resize_complex_sol(Int n) {
  sol_S_complex = CPS::VectorComp(n);
  sol_V_complex = CPS::VectorComp(n);
  sol_S_complex.setZero(n);
  sol_V_complex.setZero(n);
}

CPS::Real PFSolverPowerPolar::sol_Vr(UInt k) {
  return sol_V.coeff(k) * cos(sol_D.coeff(k));
}

CPS::Real PFSolverPowerPolar::sol_Vi(UInt k) {
  return sol_V.coeff(k) * sin(sol_D.coeff(k));
}

CPS::Complex PFSolverPowerPolar::sol_Vcx(UInt k) {
  return CPS::Complex(sol_Vr(k), sol_Vi(k));
}
