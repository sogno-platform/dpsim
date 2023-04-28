/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Solver.h>
#include <dpsim/SolverParameters.h>

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
			/// Factorization of the subnet's block
			CPS::LUFactorized luFactorization;
			/// List of all right side vector contributions
			std::vector<const Matrix*> rightVectorStamps;
			/// Left-side vector of the subnet AFTER complete step
			CPS::Attribute<Matrix>::Ptr leftVector;
		};

		///
		Real mSystemFrequency;
		/// System list
		CPS::SystemTopology mSystem;

		/// Left side vector logger
		std::shared_ptr<DataLogger> mLeftVectorLog;
		/// Right side vector logger
		std::shared_ptr<DataLogger> mRightVectorLog;

		std::vector<Subnet> mSubnets;
		std::unordered_map<typename CPS::SimNode<VarType>::Ptr, Subnet*> mNodeSubnetMap;
		typename CPS::SimPowerComp<VarType>::List mTearComponents;
		CPS::SimSignalComp::List mSimSignalComps;

		Matrix mRightSideVector;
		Matrix mLeftSideVector;
		/// Complete matrix in block form
		Matrix mSystemMatrix;
		/// Inverse of the complete system matrix (only used for initialization)
		Matrix mSystemInverse;
		/// Topology of the network removal
		Matrix mTearTopology;
		/// Impedance of the removed network
		CPS::SparseMatrixRow mTearImpedance;
		/// (Factorization of the) impedance matrix for the removed network, including
		/// the influence of other subnets
		CPS::LUFactorized mTotalTearImpedance;
		/// Currents through the removed network
		Matrix mTearCurrents;
		/// Voltages across the removed network
		Matrix mTearVoltages;

		void init(CPS::SystemTopology& system);

		void initSubnets(const std::vector<CPS::SystemTopology>& subnets);
		void collectVirtualNodes(int net);
		void assignMatrixNodeIndices(int net);
		void setSubnetSize(int net, UInt nodes);

		void setLogColumns();

		void createMatrices();
		void createTearMatrices(UInt totalSize);

		void initComponents();

		void initMatrices();
		void applyTearComponentStamp(UInt compIdx);

		void log(Real time);

	public:

		/// Currents through the removed network (as "seen" from the other subnets)
		const CPS::Attribute<Matrix>::Ptr mMappedTearCurrents;

		/// Solutions of the split systems
		const CPS::Attribute<Matrix>::Ptr mOrigLeftSideVector;

		DiakopticsSolver(String name, CPS::SystemTopology system, CPS::IdentifiedObject::List tearComponents, Real timeStep, CPS::Logger::Level logLevel);

		CPS::Task::List getTasks();

		class SubnetSolveTask : public CPS::Task {
		public:
			SubnetSolveTask(DiakopticsSolver<VarType>& solver, UInt net) :
				Task(solver.mName + ".SubnetSolve_" + std::to_string(net)), mSolver(solver), mSubnet(solver.mSubnets[net]) {
				for (auto it : mSubnet.components) {
					if (it->getRightVector()->get().size() != 0) {
						mAttributeDependencies.push_back(it->getRightVector());
					}
				}
				mModifiedAttributes.push_back(solver.mOrigLeftSideVector);
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
			Subnet& mSubnet;
		};

		// TODO better name
		class PreSolveTask : public CPS::Task {
		public:
			PreSolveTask(DiakopticsSolver<VarType>& solver) :
				Task(solver.mName + ".PreSolve"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.mOrigLeftSideVector);
				mModifiedAttributes.push_back(solver.mMappedTearCurrents);
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
		};

		class SolveTask : public CPS::Task {
		public:
			SolveTask(DiakopticsSolver<VarType>& solver, UInt net) :
				Task(solver.mName + ".Solve_" + std::to_string(net)), mSolver(solver), mSubnet(solver.mSubnets[net]) {
				mAttributeDependencies.push_back(solver.mMappedTearCurrents);
				mModifiedAttributes.push_back(mSubnet.leftVector);
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
			Subnet& mSubnet;
		};

		class PostSolveTask : public CPS::Task {
		public:
			PostSolveTask(DiakopticsSolver<VarType>& solver) :
				Task(solver.mName + ".PostSolve"), mSolver(solver) {
				for (auto& net : solver.mSubnets) {
					mAttributeDependencies.push_back(net.leftVector);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
		};

		class LogTask : public CPS::Task {
		public:
			LogTask(DiakopticsSolver<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				for (auto& net : solver.mSubnets) {
					mAttributeDependencies.push_back(net.leftVector);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
		};
	};
}
