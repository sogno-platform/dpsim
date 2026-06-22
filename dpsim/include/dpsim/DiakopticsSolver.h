/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim/DataLogger.h>
#include <dpsim/DirectLinearSolver.h>
#include <dpsim/Solver.h>

#include <atomic>
#include <unordered_map>

namespace DPsim {
template <typename VarType>
class DiakopticsSolver : public Solver, public CPS::AttributeList {
private:
  struct Subnet {
    /// Nodes assigned to this subnetwork
    typename CPS::SimNode<VarType>::List nodes;
    /// Components assigned to this subnetwork
    CPS::MNAInterface::List components;
    /// Size in system matrix (i.e. including virtual nodes)
    UInt sysSize;
    /// Offset for the imaginary part
    UInt mCmplOff;
    /// Number of real network nodes
    UInt mRealNetNodeNum;
    /// Number of virtual network nodes
    UInt mVirtualNodeNum;
    /// Offset of block in system matrix
    UInt sysOff;
    /// Sparse direct linear solver (KLU) for the subnet's block
    std::shared_ptr<DirectLinearSolver> directLinearSolver;
    /// Subnet system matrix holding the current variable-element values
    SparseMatrix systemMatrix;
    /// Index pairs of varying matrix entries within this subnet
    std::vector<std::pair<UInt, UInt>> listVariableEntries;
    /// Tear-topology columns coupled to this subnet (restrict Schur recompute)
    std::vector<UInt> tearColumns;
    /// List of all right side vector contributions
    std::vector<const Matrix *> rightVectorStamps;
    /// Left-side vector of the subnet AFTER complete step
    CPS::Attribute<Matrix>::Ptr leftVector;
    // #### MNA specific attributes related to system recomputation
    /// List of components that indicate system matrix recomputation
    CPS::MNAVariableCompInterface::List mVariableComps;
  };

  Real mSystemFrequency;
  /// System list
  CPS::SystemTopology mSystem;

  /// Left side vector logger
  std::shared_ptr<DataLogger> mLeftVectorLog;
  /// Right side vector logger
  std::shared_ptr<DataLogger> mRightVectorLog;

  std::vector<Subnet> mSubnets;
  std::unordered_map<typename CPS::SimNode<VarType>::Ptr, Subnet *>
      mNodeSubnetMap;
  typename CPS::SimPowerComp<VarType>::List mTearComponents;
  CPS::SimSignalComp::List mSimSignalComps;

  Matrix mRightSideVector;
  Matrix mLeftSideVector;
  /// Complete matrix in block form
  Matrix mSystemMatrix;
  /// Y_block^-1 * C (block-diagonal solve of the tear topology)
  Matrix mSystemInverseTearTopology;
  /// Topology of the network removal
  Matrix mTearTopology;
  /// Impedance of the removed network
  CPS::SparseMatrixRow mTearImpedance;
  /// LU factorization of the tear-impedance Schur complement
  CPS::LUFactorized mTotalTearImpedance;
  /// Tear-impedance Schur complement Z_tear + C^T * Y_block^-1 * C (un-factorized)
  Matrix mTearSchur;
  /// Set by a subnet recompute to request a Schur/LU rebuild in PreSolveTask.
  /// Written concurrently by parallel SubnetSolveTasks, so it must be atomic.
  std::atomic<bool> mTearSchurNeedsRebuild{false};
  /// Currents through the removed network
  Matrix mTearCurrents;
  /// Voltages across the removed network
  Matrix mTearVoltages;
  /// PhaseType to identify matrix Sizes
  CPS::PhaseType mPhaseType;

  void init(CPS::SystemTopology &system);

  void initSubnets(const std::vector<CPS::SystemTopology> &subnets);
  void collectVirtualNodes(int net);
  void assignMatrixNodeIndices(int net);
  void setSubnetSize(int net, UInt nodes);

  void setLogColumns();

  void createMatrices();
  void createTearMatrices(UInt totalSize);

  void initComponents();

  void initMatrices();
  void applyTearComponentStamp(UInt compIdx);

  void log(Real time, Int timeStepCount) override;

public:
  /// Currents through the removed network (as "seen" from the other subnets)
  const CPS::Attribute<Matrix>::Ptr mMappedTearCurrents;

  /// Solutions of the split systems
  const CPS::Attribute<Matrix>::Ptr mOrigLeftSideVector;

  DiakopticsSolver(String name, CPS::SystemTopology system,
                   CPS::IdentifiedObject::List tearComponents, Real timeStep,
                   CPS::Logger::Level logLevel);

  CPS::Task::List getTasks() override;

  class SubnetSolveTask : public CPS::Task {
  public:
    SubnetSolveTask(DiakopticsSolver<VarType> &solver, UInt net)
        : Task(solver.mName + ".SubnetSolve_" + std::to_string(net)),
          mSolver(solver), mSubnet(solver.mSubnets[net]) {
      for (auto it : mSubnet.components) {
        if (it->getRightVector()->get().size() != 0) {
          mAttributeDependencies.push_back(it->getRightVector());
        }
      }
      mModifiedAttributes.push_back(solver.mOrigLeftSideVector);
    }

    void execute(Real time, Int timeStepCount);
    void recomputeSubnetMatrix(Real time);
    /// Check whether status of variable MNA elements has changed
    Bool hasVariableComponentChanged();

  private:
    DiakopticsSolver<VarType> &mSolver;
    Subnet &mSubnet;
  };

  // TODO better name
  class PreSolveTask : public CPS::Task {
  public:
    PreSolveTask(DiakopticsSolver<VarType> &solver)
        : Task(solver.mName + ".PreSolve"), mSolver(solver) {
      mAttributeDependencies.push_back(solver.mOrigLeftSideVector);
      mModifiedAttributes.push_back(solver.mMappedTearCurrents);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DiakopticsSolver<VarType> &mSolver;
  };

  class SolveTask : public CPS::Task {
  public:
    SolveTask(DiakopticsSolver<VarType> &solver, UInt net)
        : Task(solver.mName + ".Solve_" + std::to_string(net)), mSolver(solver),
          mSubnet(solver.mSubnets[net]) {
      mAttributeDependencies.push_back(solver.mMappedTearCurrents);
      mModifiedAttributes.push_back(mSubnet.leftVector);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DiakopticsSolver<VarType> &mSolver;
    Subnet &mSubnet;
  };

  class PostSolveTask : public CPS::Task {
  public:
    PostSolveTask(DiakopticsSolver<VarType> &solver)
        : Task(solver.mName + ".PostSolve"), mSolver(solver) {
      for (auto &net : solver.mSubnets) {
        mAttributeDependencies.push_back(net.leftVector);
        for (UInt node = 0; node < net.mRealNetNodeNum; ++node) {
          mModifiedAttributes.push_back(net.nodes[node]->attribute("v"));
        }
      }
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DiakopticsSolver<VarType> &mSolver;
  };

  class LogTask : public CPS::Task {
  public:
    LogTask(DiakopticsSolver<VarType> &solver)
        : Task(solver.mName + ".Log"), mSolver(solver) {
      for (auto &net : solver.mSubnets) {
        mAttributeDependencies.push_back(net.leftVector);
      }
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DiakopticsSolver<VarType> &mSolver;
  };
};
} // namespace DPsim
