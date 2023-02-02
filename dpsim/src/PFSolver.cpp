/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/PFSolver.h>
#include <dpsim/SequentialScheduler.h>
#include <iostream>

using namespace DPsim;
using namespace CPS;

PFSolver::PFSolver(CPS::String name, CPS::SystemTopology system,
                   CPS::Real timeStep, CPS::Logger::Level logLevel)
    : Solver(name + "_PF", logLevel) {
  mSystem = system;
  mTimeStep = timeStep;
}

void PFSolver::initialize() {
  SPDLOG_LOGGER_INFO(mSLog, "#### INITIALIZATION OF POWERFLOW SOLVER ");
  for (auto comp : mSystem.mComponents) {
    if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
            std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp))
      mSynchronGenerators.push_back(gen);
    else if (std::shared_ptr<CPS::SP::Ph1::Load> load =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp))
      mLoads.push_back(load);
    else if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp))
      mTransformers.push_back(trafo);
    else if (std::shared_ptr<CPS::SP::Ph1::PiLine> line =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp))
      mLines.push_back(line);
    else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(
                     comp))
      mExternalGrids.push_back(extnet);
    else if (std::shared_ptr<CPS::SP::Ph1::Shunt> shunt =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::Shunt>(comp))
      mShunts.push_back(shunt);
    else if (std::shared_ptr<CPS::SP::Ph1::SolidStateTransformer> sst =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::SolidStateTransformer>(
                     comp))
      mSolidStateTransformers.push_back(sst);
    else if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi =
                 std::dynamic_pointer_cast<
                     CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
      mAverageVoltageSourceInverters.push_back(vsi);
    }
  }

  setBaseApparentPower();
  assignMatrixNodeIndices();
  initializeComponents();
  determinePFBusType();
  determineNodeBaseVoltages();
  composeAdmittanceMatrix();

  mJ.setZero(mNumUnknowns, mNumUnknowns);
  mX.setZero(mNumUnknowns);
  mF.setZero(mNumUnknowns);
}

void PFSolver::assignMatrixNodeIndices() {
  SPDLOG_LOGGER_INFO(mSLog, "Assigning simulation nodes to topology nodes:");
  UInt matrixNodeIndexIdx = 0;
  for (UInt idx = 0; idx < mSystem.mNodes.size(); ++idx) {
    mSystem.mNodes[idx]->setMatrixNodeIndex(0, matrixNodeIndexIdx);
    SPDLOG_LOGGER_INFO(mSLog, "Node {}: MatrixNodeIndex {}",
                       mSystem.mNodes[idx]->uid(),
                       mSystem.mNodes[idx]->matrixNodeIndex());
    ++matrixNodeIndexIdx;
  }
  SPDLOG_LOGGER_INFO(mSLog, "Number of simulation nodes: {:d}",
                     matrixNodeIndexIdx);
}

void PFSolver::initializeComponents() {
  for (auto comp : mSystem.mComponents) {
    std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp)
        ->updateMatrixNodeIndices();
  }

  SPDLOG_LOGGER_INFO(
      mSLog, "-- Initialize components from terminals or nodes of topology");
  for (auto comp : mSystem.mComponents) {
    auto pComp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp);
    if (!pComp)
      continue;
    if (mInitFromNodesAndTerminals)
      pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "-- Calculate per unit parameters for all components");
  for (auto extnet : mExternalGrids) {
    extnet->calculatePerUnitParameters(mBaseApparentPower,
                                       mSystem.mSystemOmega);
  }
  for (auto line : mLines) {
    line->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
  }
  for (auto trans : mTransformers) {
    trans->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
  }
  for (auto shunt : mShunts) {
    shunt->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
  }
  for (auto load : mLoads) {
    load->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
  }
  for (auto gen : mSynchronGenerators) {
    gen->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
  }
  for (auto sst : mSolidStateTransformers) {
    sst->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
  }
}

void PFSolver::setBaseApparentPower() {
  Real maxPower = 0.;
  if (!mSynchronGenerators.empty()) {
    for (auto gen : mSynchronGenerators)
      if (std::abs(gen->attributeTyped<Real>("P_set")->get()) > maxPower)
        maxPower = std::abs(gen->attributeTyped<Real>("P_set")->get());
  } else if (!mTransformers.empty()) {
    for (auto trafo : mTransformers)
      if (trafo->attributeTyped<Real>("S")->get() > maxPower)
        maxPower = trafo->attributeTyped<Real>("S")->get();
  }
  if (maxPower != 0.)
    mBaseApparentPower = pow(10, 1 + floor(log10(maxPower)));
  else {
    mBaseApparentPower = 100000000;
    SPDLOG_LOGGER_WARN(mSLog,
                       "No suitable quantity found for setting "
                       "mBaseApparentPower. Using {} VA.",
                       mBaseApparentPower);
  }
  SPDLOG_LOGGER_INFO(mSLog, "Base power = {} VA", mBaseApparentPower);
}

void PFSolver::determinePFBusType() {
  mPQBusIndices.clear();
  mPVBusIndices.clear();
  mVDBusIndices.clear();

  SPDLOG_LOGGER_INFO(mSLog, "-- Determine powerflow bus type for each node");

  // Determine powerflow bus type of each node through analysis of system topology
  for (auto node : mSystem.mNodes) {
    bool connectedPV = false;
    bool connectedPQ = false;
    bool connectedVD = false;

    for (auto comp : mSystem.mComponentsAtNode[node]) {
      if (std::shared_ptr<CPS::SP::Ph1::Load> load =
              std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
        if (load->mPowerflowBusType == CPS::PowerflowBusType::PQ) {
          connectedPQ = true;
        }
      } else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                         comp)) {
        if (gen->mPowerflowBusType == CPS::PowerflowBusType::PV) {
          connectedPV = true;
        } else if (gen->mPowerflowBusType == CPS::PowerflowBusType::VD) {
          connectedVD = true;
        }
      } else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(
                         comp)) {
        if (extnet->mPowerflowBusType == CPS::PowerflowBusType::VD) {
          connectedVD = true;
        } else if (extnet->mPowerflowBusType == CPS::PowerflowBusType::PV) {
          connectedPV = true;
        }
      }
    }

    // determine powerflow bus types according connected type of connected components
    // only PQ type component connected -> set as PQ bus
    if (!connectedPV && connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_DEBUG(
          mSLog, "{}: only PQ type component connected -> set as PQ bus",
          node->name());
      mPQBusIndices.push_back(node->matrixNodeIndex());
      mPQBuses.push_back(node);
    } // no component connected -> set as PQ bus (P & Q will be zero)
    else if (!connectedPV && !connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_DEBUG(mSLog, "{}: no component connected -> set as PQ bus",
                          node->name());
      mPQBusIndices.push_back(node->matrixNodeIndex());
      mPQBuses.push_back(node);
    } // only PV type component connected -> set as PV bus
    else if (connectedPV && !connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_DEBUG(
          mSLog, "{}: only PV type component connected -> set as PV bus",
          node->name());
      mPVBusIndices.push_back(node->matrixNodeIndex());
      mPVBuses.push_back(node);
    } // PV and PQ type component connected -> set as PV bus (TODO: bus type should be modifiable by user afterwards)
    else if (connectedPV && connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_DEBUG(
          mSLog, "{}: PV and PQ type component connected -> set as PV bus",
          node->name());
      mPVBusIndices.push_back(node->matrixNodeIndex());
      mPVBuses.push_back(node);
    } // only VD type component connected -> set as VD bus
    else if (!connectedPV && !connectedPQ && connectedVD) {
      SPDLOG_LOGGER_DEBUG(
          mSLog, "{}: only VD type component connected -> set as VD bus",
          node->name());
      mVDBusIndices.push_back(node->matrixNodeIndex());
      mVDBuses.push_back(node);
    } // VD and PV type component connect -> set as VD bus
    else if (connectedPV && !connectedPQ && connectedVD) {
      SPDLOG_LOGGER_DEBUG(
          mSLog, "{}: VD and PV type component connect -> set as VD bus",
          node->name());
      mVDBusIndices.push_back(node->matrixNodeIndex());
      mVDBuses.push_back(node);
    } // VD, PV and PQ type component connect -> set as VD bus
    else if (connectedPV && connectedPQ && connectedVD) {
      SPDLOG_LOGGER_DEBUG(
          mSLog, "{}: VD, PV and PQ type component connect -> set as VD bus",
          node->name());
      mVDBusIndices.push_back(node->matrixNodeIndex());
      mVDBuses.push_back(node);
    } else {
      std::stringstream ss;
      ss << "Node>>" << node->name()
         << ": combination of connected components is invalid";
      throw std::invalid_argument(ss.str());
    }
  }

  mNumPQBuses = mPQBusIndices.size();
  mNumPVBuses = mPVBusIndices.size();
  mNumVDBuses = mVDBusIndices.size();
  mNumUnknowns = 2 * mNumPQBuses + mNumPVBuses;

  // Aggregate PQ bus and PV bus index vectors for easy handling in solver
  mPQPVBusIndices.reserve(mNumPQBuses + mNumPVBuses);
  mPQPVBusIndices.insert(mPQPVBusIndices.end(), mPQBusIndices.begin(),
                         mPQBusIndices.end());
  mPQPVBusIndices.insert(mPQPVBusIndices.end(), mPVBusIndices.begin(),
                         mPVBusIndices.end());

  SPDLOG_LOGGER_INFO(mSLog, "#### Create index vectors for power flow solver:");
  SPDLOG_LOGGER_INFO(mSLog, "PQ Buses: {}", logVector(mPQBusIndices));
  SPDLOG_LOGGER_INFO(mSLog, "PV Buses: {}", logVector(mPVBusIndices));
  SPDLOG_LOGGER_INFO(mSLog, "VD Buses: {}", logVector(mVDBusIndices));
}

void PFSolver::determineNodeBaseVoltages() {

    SPDLOG_LOGGER_INFO(mSLog, "-- Determine base voltages for each node according to connected components");
    mSLog->flush();

	for (auto node : mSystem.mNodes) {
		CPS::Real baseVoltage_ = 0;
		for (auto comp : mSystem.mComponentsAtNode[node]) {
            if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi = std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
				baseVoltage_=Math::abs(vsi->attributeTyped<CPS::Complex>("vnom")->get());
                SPDLOG_LOGGER_INFO(mSLog, "Choose base voltage {}V of {} to convert pu-solution of {}.", baseVoltage_, vsi->name(), node->name());
                break;
			}
            else if (std::shared_ptr<CPS::SP::Ph1::RXLine> rxline = std::dynamic_pointer_cast<CPS::SP::Ph1::RXLine>(comp)) {
				baseVoltage_ = rxline->attributeTyped<CPS::Real>("base_Voltage")->get();
                SPDLOG_LOGGER_INFO(mSLog, "Choose base voltage {}V of {} to convert pu-solution of {}.", baseVoltage_, rxline->name(), node->name());
                break;
			}
            else if (std::shared_ptr<CPS::SP::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp)) {
				baseVoltage_ = line->attributeTyped<CPS::Real>("base_Voltage")->get();
                SPDLOG_LOGGER_INFO(mSLog, "Choose base voltage {}V of {} to convert pu-solution of {}.", baseVoltage_, line->name(), node->name());
                break;
			}
			else if (std::shared_ptr<CPS::SP::Ph1::Transformer> trans = std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp)) {
				if (trans->terminal(0)->node()->name() == node->name()){
                    baseVoltage_ = trans->attributeTyped<CPS::Real>("nominal_voltage_end1")->get();
                    SPDLOG_LOGGER_INFO(mSLog, "Choose base voltage {}V of {} to convert pu-solution of {}.", baseVoltage_, trans->name(), node->name());
                    break;
                }
				else if (trans->terminal(1)->node()->name() == node->name()){
                    baseVoltage_ = trans->attributeTyped<CPS::Real>("nominal_voltage_end2")->get();
                    SPDLOG_LOGGER_INFO(mSLog, "Choose base voltage {}V of {} to convert pu-solution of {}.", baseVoltage_, trans->name(), node->name());
                    break;
                }
            }
            else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
                    baseVoltage_ = gen->attributeTyped<CPS::Real>("base_Voltage")->get();
                    SPDLOG_LOGGER_INFO(mSLog, "Choose base voltage {}V of {} to convert pu-solution of {}.", baseVoltage_, gen->name(), node->name());
                    break;
                }
			else if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
                    baseVoltage_ = load->attributeTyped<CPS::Real>("V_nom")->get();
                    mSLog->info("Choose base voltage of {}V to convert pu-solution of {}.", baseVoltage_, load->name(), node->name());
                    break;
                }
			else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet = std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
                    baseVoltage_ = extnet->attributeTyped<CPS::Real>("base_Voltage")->get();
                    mSLog->info("Choose base voltage of {}V to convert pu-solution of {}.", baseVoltage_, extnet->name(), node->name());
                    break;
                }
            else {
                SPDLOG_LOGGER_WARN(mSLog, "Unable to get base voltage at {}", node->name());
                }
        }
      } else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
                     std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                         comp)) {
        baseVoltage_ = gen->attributeTyped<CPS::Real>("base_Voltage")->get();
        SPDLOG_LOGGER_INFO(
            mSLog,
            "Choose base voltage {}V of {} to convert pu-solution of {}.",
            baseVoltage_, gen->name(), node->name());
        break;
      } else {
        SPDLOG_LOGGER_WARN(mSLog, "Unable to get base voltage at {}",
                           node->name());
      }
    }
    mBaseVoltageAtNode[node] = baseVoltage_;
  }
}

void PFSolver::setVDNode(CPS::String name) {
  if (!mExternalGrids.empty()) {
    if (mExternalGrids[0]->node(0)->name() == name) {
      mExternalGrids[0]->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
    }
  } else {
    for (auto gen : mSynchronGenerators) {
      if (gen->node(0)->name() == name) {
        gen->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
        return;
      }
    }
    throw std::invalid_argument("Invalid slack bus, no external grid or "
                                "synchronous generator attached");
  }
}

void PFSolver::modifyPowerFlowBusComponent(
    CPS::String name, CPS::PowerflowBusType powerFlowBusType) {
  for (auto comp : mSystem.mComponents) {
    if (comp->name() == name) {
      if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
              std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp))
        extnet->modifyPowerFlowBusType(powerFlowBusType);
      else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen =
                   std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(
                       comp))
        gen->modifyPowerFlowBusType(powerFlowBusType);
    }
  }
}

void PFSolver::setSolverAndComponentBehaviour(Solver::Behaviour behaviour) {
  mBehaviour = behaviour;
  if (mBehaviour == Behaviour::Initialization) {
    SPDLOG_LOGGER_INFO(mSLog, "-- Set solver behaviour to Initialization");
    // TODO: solver setting specific to initialization (e.g. one single PF run)

    SPDLOG_LOGGER_INFO(mSLog, "-- Set component behaviour to Initialization");
    for (auto comp : mSystem.mComponents) {
      auto powerComp =
          std::dynamic_pointer_cast<CPS::TopologicalPowerComp>(comp);
      if (powerComp)
        powerComp->setBehaviour(
            TopologicalPowerComp::Behaviour::Initialization);
    }
  } else {
    SPDLOG_LOGGER_INFO(mSLog, "-- Set solver behaviour to Simulation");
    // TODO: solver setting specific to simulation

    SPDLOG_LOGGER_INFO(mSLog, "-- Set component behaviour to PFSimulation");
    for (auto comp : mSystem.mComponents) {
      auto powerComp =
          std::dynamic_pointer_cast<CPS::TopologicalPowerComp>(comp);
      if (powerComp)
        powerComp->setBehaviour(TopologicalPowerComp::Behaviour::PFSimulation);
    }
  }
}

void PFSolver::composeAdmittanceMatrix() {
  int n = mSystem.mNodes.size();
  if (n > 0) {
    mY = CPS::SparseMatrixComp(n, n);
    for (auto line : mLines) {
      line->pfApplyAdmittanceMatrixStamp(mY);
    }
    for (auto trans : mTransformers) {
      //to check if this transformer could be ignored
      if (**trans->mResistance == 0 && **trans->mInductance == 0) {
        SPDLOG_LOGGER_INFO(mSLog, "{} {} ignored for R = 0 and L = 0",
                           trans->type(), trans->name());
        continue;
      }
      trans->pfApplyAdmittanceMatrixStamp(mY);
    }
    for (auto shunt : mShunts) {
      shunt->pfApplyAdmittanceMatrixStamp(mY);
    }
  }
  if (mLines.empty() && mTransformers.empty()) {
    throw std::invalid_argument("There are no bus");
  }
}

CPS::Real PFSolver::G(int i, int j) { return mY.coeff(i, j).real(); }

CPS::Real PFSolver::B(int i, int j) { return mY.coeff(i, j).imag(); }

CPS::Bool PFSolver::checkConvergence() {
  // Converged if all mismatches are below the tolerance
  for (CPS::UInt i = 0; i < mNumUnknowns; i++) {
    if (abs(mF(i)) > mTolerance)
      return false;
  }
  return true;
}

Bool PFSolver::solvePowerflow() {
  // Calculate the mismatch according to the initial solution
  calculateMismatch();

  // Check whether model already converged
  isConverged = checkConvergence();

  mIterations = 0;
  for (unsigned i = 1; i < mMaxIterations && !isConverged; ++i) {

    calculateJacobian();
    auto sparseJ = mJ.sparseView();

    // Solve system mJ*mX = mF
    Eigen::SparseLU<SparseMatrix> lu(sparseJ);

    mX = lu.solve(mF); /* code */

    // Calculate new solution based on mX increments obtained from equation system
    updateSolution();

    // Calculate the mismatch according to the current solution
    calculateMismatch();

    SPDLOG_LOGGER_DEBUG(mSLog, "Mismatch vector at iteration {}: \n {}", i, mF);
    mSLog->flush();

    // Check convergence
    isConverged = checkConvergence();
    mIterations = i;
  }
  return isConverged;
}

void PFSolver::SolveTask::execute(Real time, Int timeStepCount) {
  // apply keepLastSolution to save computation time
  mSolver.generateInitialSolution(time);
  mSolver.solvePowerflow();
  mSolver.setSolution();
}

Task::List PFSolver::getTasks() {
  return Task::List{std::make_shared<SolveTask>(*this)};
}
