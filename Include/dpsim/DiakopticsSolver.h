/** Diakoptics solver
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <cps/AttributeList.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/SignalComponent.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Solver.h>

#include <unordered_map>

namespace DPsim {
	template <typename VarType>
	class DiakopticsSolver : public Solver, public CPS::AttributeList {
	private:
		struct Subnet {
			/// Nodes assigned to this subnetwork
			typename CPS::Node<VarType>::List nodes;
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

		String mName;
		Real mTimeStep;
		Real mSystemFrequency;
		/// System list
		CPS::SystemTopology mSystem;

		/// Left side vector logger
		std::shared_ptr<DataLogger> mLeftVectorLog;
		/// Right side vector logger
		std::shared_ptr<DataLogger> mRightVectorLog;

		std::vector<Subnet> mSubnets;
		std::unordered_map<typename CPS::Node<VarType>::Ptr, Subnet*> mNodeSubnetMap;
		typename CPS::PowerComponent<VarType>::List mTearComponents;
		CPS::SignalComponent::List mSignalComponents;

		Matrix mRightSideVector;
		Matrix mLeftSideVector;
		/// Complete matrix in block form
		Matrix mSystemMatrix;
		/// Inverse of the complete system matrix (only used for initialization)
		Matrix mSystemInverse;
		/// Solutions of the split systems
		Matrix mOrigLeftSideVector;
		/// Topology of the network removal
		Matrix mTearTopology;
		/// Impedance of the removed network
		Matrix mTearImpedance;
		/// (Factorization of the) impedance matrix for the removed network, including
		/// the influence of other subnets
		CPS::LUFactorized mTotalTearImpedance;
		/// Currents through the removed network
		Matrix mTearCurrents;
		/// Currents through the removed network (as "seen" from the other subnets)
		Matrix mMappedTearCurrents;
		/// Voltages across the removed network
		Matrix mTearVoltages;

		void init(CPS::SystemTopology& system);

		void initSubnets(const std::vector<CPS::SystemTopology>& subnets);
		void collectVirtualNodes(int net);
		void assignSimNodes(int net);
		void setSubnetSize(int net, UInt nodes);

		void setLogColumns();

		void createMatrices();
		void createTearMatrices(UInt totalSize);

		void initComponents();

		void initMatrices();
		void applyTearComponentStamp(UInt compIdx);

		void log(Real time);

	public:
		DiakopticsSolver(String name, CPS::SystemTopology system, CPS::Component::List tearComponents, Real timeStep, CPS::Logger::Level logLevel);

		CPS::Task::List getTasks();

		class SubnetSolveTask : public CPS::Task {
		public:
			SubnetSolveTask(DiakopticsSolver<VarType>& solver, UInt net) :
				Task(solver.mName + ".SubnetSolve_" + std::to_string(net)), mSolver(solver), mSubnet(solver.mSubnets[net]) {
				for (auto it : mSubnet.components) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0) {
						mAttributeDependencies.push_back(it->attribute("right_vector"));
					}
				}
				mModifiedAttributes.push_back(solver.attribute("old_left_vector"));
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
				mAttributeDependencies.push_back(solver.attribute("old_left_vector"));
				mModifiedAttributes.push_back(solver.attribute("mapped_tear_currents"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
		};

		class SolveTask : public CPS::Task {
		public:
			SolveTask(DiakopticsSolver<VarType>& solver, UInt net) :
				Task(solver.mName + ".Solve_" + std::to_string(net)), mSolver(solver), mSubnet(solver.mSubnets[net]) {
				mAttributeDependencies.push_back(solver.attribute("mapped_tear_currents"));
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
