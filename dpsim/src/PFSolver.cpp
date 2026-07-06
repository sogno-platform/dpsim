/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/MathUtils.h>
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
  propagateAndVerifyBaseVoltage();
  composeAdmittanceMatrix();

  setUpJacobianStorage();
  mX.setZero(mNumUnknowns);
  mF.setZero(mNumUnknowns);
}

void PFSolver::setUpJacobianStorage() {
  mJ.setZero(mNumUnknowns, mNumUnknowns);
}

void PFSolver::solveJacobianSystem() {
  auto sparseJ = mJ.sparseView();
  Eigen::SparseLU<SparseMatrix> lu(sparseJ);
  mX = lu.solve(mF);
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
    mBaseApparentPower = mBaseApparentPowerFallback;
    SPDLOG_LOGGER_WARN(mSLog,
                       "No suitable quantity found for setting "
                       "mBaseApparentPower. Using {} VA.",
                       mBaseApparentPower);
  }
  SPDLOG_LOGGER_INFO(mSLog, "Base power = {} VA", mBaseApparentPower);
}

void PFSolver::determinePFBusType() {
  mPQBuses.clear();
  mPVBuses.clear();
  mVDBuses.clear();

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
        } else if (gen->mPowerflowBusType == CPS::PowerflowBusType::PQ) {
          connectedPQ = true;
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
      SPDLOG_LOGGER_INFO(
          mSLog, "{}: only PQ type component connected -> set as PQ bus",
          node->name());
      mPQBuses.push_back(node);
    } // no component connected -> set as PQ bus (P & Q will be zero)
    else if (!connectedPV && !connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_INFO(mSLog, "{}: no component connected -> set as PQ bus",
                         node->name());
      mPQBuses.push_back(node);
    } // only PV type component connected -> set as PV bus
    else if (connectedPV && !connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_INFO(
          mSLog, "{}: only PV type component connected -> set as PV bus",
          node->name());
      mPVBuses.push_back(node);
    } // PV and PQ type component connected -> set as PV bus (TODO: bus type should be modifiable by user afterwards)
    else if (connectedPV && connectedPQ && !connectedVD) {
      SPDLOG_LOGGER_INFO(
          mSLog, "{}: PV and PQ type component connected -> set as PV bus",
          node->name());
      mPVBuses.push_back(node);
    } // only VD type component connected -> set as VD bus
    else if (!connectedPV && !connectedPQ && connectedVD) {
      SPDLOG_LOGGER_INFO(
          mSLog, "{}: only VD type component connected -> set as VD bus",
          node->name());
      mVDBuses.push_back(node);
    } // VD and PV type component connect -> set as VD bus
    else if (connectedPV && !connectedPQ && connectedVD) {
      SPDLOG_LOGGER_INFO(
          mSLog, "{}: VD and PV type component connect -> set as VD bus",
          node->name());
      mVDBuses.push_back(node);
    } // VD, PV and PQ type component connect -> set as VD bus
    else if (connectedPV && connectedPQ && connectedVD) {
      SPDLOG_LOGGER_INFO(
          mSLog, "{}: VD, PV and PQ type component connect -> set as VD bus",
          node->name());
      mVDBuses.push_back(node);
    } else {
      std::stringstream ss;
      ss << "Node>>" << node->name()
         << ": combination of connected components is invalid";
      throw std::invalid_argument(ss.str());
    }
  }

  rebuildBusIndexAggregates();

  // Snapshot so each solve can reset before Q-limit switching (solver is reused).
  mPQBusesOrig = mPQBuses;
  mPVBusesOrig = mPVBuses;

  SPDLOG_LOGGER_INFO(mSLog, "#### Create index vectors for power flow solver:");
  SPDLOG_LOGGER_INFO(mSLog, "PQ Buses: {}", logVector(mPQBusIndices));
  SPDLOG_LOGGER_INFO(mSLog, "PV Buses: {}", logVector(mPVBusIndices));
  SPDLOG_LOGGER_INFO(mSLog, "VD Buses: {}", logVector(mVDBusIndices));
}

void PFSolver::rebuildBusIndexAggregates() {
  // Rebuild index vectors from the PQ/PV/VD lists (initial + after each Q-limit switch).
  mPQBusIndices.clear();
  mPVBusIndices.clear();
  mVDBusIndices.clear();
  for (auto node : mPQBuses)
    mPQBusIndices.push_back(node->matrixNodeIndex());
  for (auto node : mPVBuses)
    mPVBusIndices.push_back(node->matrixNodeIndex());
  for (auto node : mVDBuses)
    mVDBusIndices.push_back(node->matrixNodeIndex());

  mNumPQBuses = mPQBusIndices.size();
  mNumPVBuses = mPVBusIndices.size();
  mNumVDBuses = mVDBusIndices.size();
  mNumUnknowns = 2 * mNumPQBuses + mNumPVBuses;

  // Aggregate PQ bus and PV bus index vectors for easy handling in solver
  mPQPVBusIndices.clear();
  mPQPVBusIndices.reserve(mNumPQBuses + mNumPVBuses);
  mPQPVBusIndices.insert(mPQPVBusIndices.end(), mPQBusIndices.begin(),
                         mPQBusIndices.end());
  mPQPVBusIndices.insert(mPQPVBusIndices.end(), mPVBusIndices.begin(),
                         mPVBusIndices.end());
}

void PFSolver::resetToOriginalClassification() {
  // Reset to the pre-switching classification so a fresh solve starts clean.
  mPQBuses = mPQBusesOrig;
  mPVBuses = mPVBusesOrig;
  clearReactiveLimitState();
  reclassifyBuses();
}

void PFSolver::reclassifyBuses() {
  // Re-derive index vectors and resize storage; sol_V/sol_D carry over as a warm start.
  rebuildBusIndexAggregates();
  setUpJacobianStorage();
  mX.setZero(mNumUnknowns);
  mF.setZero(mNumUnknowns);
}

CPS::Real PFSolver::componentBaseVoltage(CPS::TopologicalPowerComp::Ptr comp,
                                         CPS::TopologicalNode::Ptr node) {
  if (auto vsi =
          std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(
              comp))
    return vsi->getBaseVoltage();
  if (auto rxline = std::dynamic_pointer_cast<CPS::SP::Ph1::RXLine>(comp))
    return rxline->getBaseVoltage();
  if (auto line = std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp))
    return line->getBaseVoltage();
  if (auto trans = std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp)) {
    if (trans->terminal(0)->node()->name() == node->name())
      return trans->getNominalVoltageEnd1();
    if (trans->terminal(1)->node()->name() == node->name())
      return trans->getNominalVoltageEnd2();
    return 0;
  }
  if (auto gen =
          std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp))
    return gen->getBaseVoltage();
  if (auto load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp))
    return load->getNomVoltage();
  if (auto extnet =
          std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp))
    return extnet->getBaseVoltage();
  if (auto shunt = std::dynamic_pointer_cast<CPS::SP::Ph1::Shunt>(comp))
    return shunt->getBaseVoltage();
  SPDLOG_LOGGER_WARN(mSLog, "Unable to get base voltage at {}", node->name());
  return 0;
}

void PFSolver::propagateAndVerifyBaseVoltage() {

  SPDLOG_LOGGER_INFO(mSLog, "-- Determine base voltages for each node "
                            "according to connected components");
  mSLog->flush();

  // Zones: nodes joined by a line share one voltage level; transformers are boundaries.
  std::vector<UInt> zoneParent(mSystem.mNodes.size());
  for (UInt i = 0; i < zoneParent.size(); ++i)
    zoneParent[i] = i;
  auto findZone = [&](UInt node) -> UInt {
    while (zoneParent[node] != node) {
      zoneParent[node] = zoneParent[zoneParent[node]];
      node = zoneParent[node];
    }
    return node;
  };
  auto uniteZones = [&](UInt a, UInt b) {
    zoneParent[findZone(a)] = findZone(b);
  };

  for (auto comp : mSystem.mComponents) {
    if (auto line = std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp))
      uniteZones(line->node(0)->matrixNodeIndex(),
                 line->node(1)->matrixNodeIndex());
    else if (auto rxline =
                 std::dynamic_pointer_cast<CPS::SP::Ph1::RXLine>(comp))
      uniteZones(rxline->node(0)->matrixNodeIndex(),
                 rxline->node(1)->matrixNodeIndex());
  }

  // Generator/Transformer/NetworkInjection/VSI ratings are authoritative;
  // everything else (incl. Load's solved-voltage proxy) is a looser fallback.
  std::map<UInt, std::vector<std::pair<CPS::Real, CPS::String>>> authoritative;
  std::map<UInt, std::vector<std::pair<CPS::Real, CPS::String>>> fallback;
  std::map<UInt, std::vector<std::shared_ptr<CPS::SP::Ph1::Load>>> zoneLoads;
  for (auto node : mSystem.mNodes) {
    UInt zone = findZone(node->matrixNodeIndex());
    for (auto comp : mSystem.mComponentsAtNode[node]) {
      if (auto load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp))
        zoneLoads[zone].push_back(load);

      CPS::Real voltage = componentBaseVoltage(comp, node);
      if (std::abs(voltage) <= 1e-6)
        continue;
      bool isAuthoritative =
          std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp) ||
          std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp) ||
          std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp) ||
          std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(
              comp);
      auto &bucket = isAuthoritative ? authoritative : fallback;
      bucket[zone].emplace_back(voltage, comp->name());
    }
  }

  // Disagreement beyond tolerance means two voltage levels are wired together without a transformer.
  static constexpr CPS::Real strictTolerance = 0.01;
  static constexpr CPS::Real looseTolerance = 0.1;
  auto verify =
      [&](const std::vector<std::pair<CPS::Real, CPS::String>> &candidates,
          CPS::Real reference, const CPS::String &refSource,
          CPS::Real tolerance) {
        for (auto &candidate : candidates) {
          CPS::Real relDiff =
              std::abs(candidate.first - reference) /
              std::max(std::abs(candidate.first), std::abs(reference));
          if (relDiff > tolerance) {
            std::stringstream ss;
            ss << "Base voltage mismatch within one electrical zone (nodes "
                  "connected without an intervening transformer): "
               << refSource << " implies " << reference << "V but "
               << candidate.second << " implies " << candidate.first << "V";
            throw std::invalid_argument(ss.str());
          }
        }
      };

  std::map<UInt, CPS::Real> zoneVoltage;
  for (auto &entry : authoritative) {
    CPS::Real refVoltage = entry.second.front().first;
    verify(entry.second, refVoltage, entry.second.front().second,
           strictTolerance);
    zoneVoltage[entry.first] = refVoltage;
  }
  // Fallback checked against the zone's rating, or each other if there is none.
  for (auto &entry : fallback) {
    auto it = zoneVoltage.find(entry.first);
    bool hasAuthoritative = it != zoneVoltage.end();
    CPS::Real reference =
        hasAuthoritative ? it->second : entry.second.front().first;
    const CPS::String &refSource = hasAuthoritative
                                       ? "the zone's authoritative rating"
                                       : entry.second.front().second;
    verify(entry.second, reference, refSource, looseTolerance);
    if (!hasAuthoritative)
      zoneVoltage[entry.first] = reference;
  }

  // Assign the resolved zone voltage to every node in it.
  for (auto node : mSystem.mNodes) {
    auto it = zoneVoltage.find(findZone(node->matrixNodeIndex()));
    mBaseVoltageAtNode[node] = it != zoneVoltage.end() ? it->second : 0;
  }

  // Sync each Load's nominal voltage to its zone's resolved value.
  for (auto &entry : zoneVoltage) {
    auto it = zoneLoads.find(entry.first);
    if (it == zoneLoads.end())
      continue;
    for (auto &load : it->second) {
      if (std::abs(load->getNomVoltage() - entry.second) > 1e-6)
        load->setParameters(load->attributeTyped<CPS::Real>("P")->get(),
                            load->attributeTyped<CPS::Real>("Q")->get(),
                            entry.second);
    }
  }

  UInt numMissing = 0;
  UInt numZero = 0;

  for (auto node : mSystem.mNodes) {

    auto it = mBaseVoltageAtNode.find(node);

    if (it == mBaseVoltageAtNode.end()) {
      SPDLOG_LOGGER_WARN(mSLog, "No base voltage entry for {}", node->name());

      numMissing++;
      continue;
    }

    if (std::abs(it->second) < 1e-6) {
      SPDLOG_LOGGER_WARN(mSLog, "Zero base voltage for {}", node->name());

      numZero++;
    }
  }

  SPDLOG_LOGGER_INFO(mSLog, "Base voltage summary: missing={}, zero={}",
                     numMissing, numZero);
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
    if (!Math::isFinite(mF(i))) {
      SPDLOG_LOGGER_WARN(mSLog, "mF[{}] not finite (NaN/Inf)", i);
      return false;
    }
    if (abs(mF(i)) > mTolerance)
      return false;
  }
  return true;
}

Bool PFSolver::runNewtonRaphson() {

  // Reset values for new power flow run
  isConverged = false;
  mIterations = 0;
  mX.setZero();
  mF.setZero();

  // Calculate the mismatch according to the initial solution
  calculateMismatch();

  // Check whether model already converged
  isConverged = checkConvergence();

  for (unsigned i = 1; i < mMaxIterations && !isConverged; ++i) {

    calculateJacobian();

    // Solve system mJ*mX = mF
    solveJacobianSystem();

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

Bool PFSolver::solvePowerflow() {
  Bool converged = runNewtonRaphson();

  if (!mEnforceReactiveLimits)
    return converged;

  // Outer loop: switch PV<->PQ on Q-limit violations, re-solve until no bus switches.
  Bool settled = false;
  for (CPS::UInt outer = 0; converged && outer < mMaxOuterIterations; ++outer) {
    if (!enforceReactiveLimits()) {
      settled = true;
      break; // all generators within their reactive limits
    }
    reclassifyBuses();
    converged = runNewtonRaphson();
  }

  if (converged && !settled) {
    // Unsettled PV/PQ classification must not look converged to setSolution().
    SPDLOG_LOGGER_WARN(
        mSLog,
        "Q-limit outer loop did not settle within {} iterations; "
        "PV/PQ classification may still be oscillating",
        mMaxOuterIterations);
    isConverged = false;
    converged = false;
  }
  return converged;
}

void PFSolver::SolveTask::execute(Real time, Int timeStepCount) {
  // apply keepLastSolution to save computation time
  mSolver.generateInitialSolution(time, mSolver.mKeepLastSolution);
  mSolver.solvePowerflow();
  mSolver.setSolution();
}

Task::List PFSolver::getTasks() {
  return Task::List{std::make_shared<SolveTask>(*this)};
}
