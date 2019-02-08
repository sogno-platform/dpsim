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
			typename CPS::Node<VarType>::List nodes;
			CPS::MNAInterface::List components;
			/// Size in system matrix (i.e. including virtual nodes)
			UInt sysSize;
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

		CPS::Logger mLog;
		/// Left side vector logger
		DataLogger mLeftVectorLog;
		/// Right side vector logger
		DataLogger mRightVectorLog;

		std::vector<Subnet> mSubnets;
		std::unordered_map<typename CPS::Node<VarType>::Ptr, Subnet*> mNodeSubnetMap;
		typename CPS::PowerComponent<VarType>::List mTearComponents;
		CPS::SignalComponent::List mSignalComponents;

		/// Complete matrix in block form
		Matrix mSystemMatrix;
		/// XXX shouldn't be explicitly calculated
		Matrix mYinv;
		Matrix mRightSideVector;
		/// Solutions of the split systems
		Matrix mOrigLeftSideVector;
		Matrix mLeftSideVector;
		Matrix mTearTopology;
		Matrix mRemovedImpedance;
		CPS::LUFactorized mNetToRemovedImpedance;
		// TODO better names
		Matrix mIPsi;
		Matrix mIPsiMapped;
		Matrix mEPsi;

		void init(const CPS::SystemTopology& system);

		void initSubnets(const std::vector<CPS::SystemTopology>& subnets);
		void createVirtualNodes(int net);
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
				mModifiedAttributes.push_back(solver.attribute("xOld"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			DiakopticsSolver<VarType>& mSolver;
			Subnet& mSubnet;
		};

		class SolveTask : public CPS::Task {
		public:
			SolveTask(DiakopticsSolver<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("xOld"));
				for (auto& net : solver.mSubnets) {
					mModifiedAttributes.push_back(net.leftVector);
				}
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
