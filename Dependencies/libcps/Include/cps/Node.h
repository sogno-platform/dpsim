/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <cps/TopologicalNode.h>
#include <cps/Task.h>

namespace CPS {

	template <typename VarType>
	class Node : 	public TopologicalNode,
					public std::enable_shared_from_this<Node<VarType>>,
					public SharedFactory<Node<VarType>> {
	protected:
		///
		std::vector<UInt> mSimNode = { 0 };
		/// List of considered network harmonics
		Matrix mFrequencies;
		/// Number of harmonics
		UInt mNumFreqs = 0;
		///
		MatrixVar<VarType> mVoltage;
		///
		Task::List mMnaTasks;
	public:
		typedef VarType Type;
		typedef std::shared_ptr<Node<VarType>> Ptr;
		typedef std::vector<Ptr> List;
		///
		static Ptr GND;

		/// This very general constructor is used by other constructors.
		Node(String name, String uid, std::vector<UInt> simNode,
			PhaseType phaseType, std::vector<Complex> initialVoltage);
		/// Create ground node if no parameters are given.
		Node(PhaseType phaseType = PhaseType::Single);
		/// Create named node and optionally assigns an initial voltage.
		/// This should be the constructor called by users in examples.
		Node(String name, PhaseType phaseType = PhaseType::Single,
			std::vector<Complex> initialVoltage = { 0, 0, 0 })
			: Node(name, name, { 0, 0, 0 }, phaseType, initialVoltage) { }
		/// Create node with name and node number.
		/// This is mostly used by functions.
		Node(String uid, String name, UInt simNode,
			PhaseType phaseType = PhaseType::Single, std::vector<Complex> initialVoltage = { 0, 0, 0 })
			: Node(uid, name, { simNode, simNode + 1, simNode + 2 }, phaseType, initialVoltage) {}
		/// Create node with default name and node number.
		/// This is mostly used by functions.
		Node(UInt simNode, PhaseType phaseType = PhaseType::Single)
			: Node("N" + std::to_string(simNode), "N" + std::to_string(simNode), simNode, phaseType) { }

		/// Initialize state matrices with size according to phase type and frequency number
		void initialize(Matrix frequencies);
		/// Returns matrix index for specified phase
		UInt simNode(PhaseType phaseType = PhaseType::Single) {
			if ((phaseType == PhaseType::A || phaseType == PhaseType::Single)
				&& (mPhaseType == PhaseType::Single
				|| mPhaseType == PhaseType::A
				|| mPhaseType == PhaseType::ABC))
				return mSimNode[0];
			else if (phaseType == PhaseType::B
				&& (mPhaseType == PhaseType::B
				|| mPhaseType == PhaseType::ABC))
				return mSimNode[1];
			else if (phaseType == PhaseType::C
				&& (mPhaseType == PhaseType::C
				|| mPhaseType == PhaseType::ABC))
				return mSimNode[2];
			else
				return 0;
		}
		/// Returns all matrix indices
		std::vector<UInt> simNodes() {
			if (mPhaseType == PhaseType::B)
				return { mSimNode[1] };
			else if (mPhaseType == PhaseType::C)
				return { mSimNode[2] };
			else if (mPhaseType == PhaseType::ABC)
				return mSimNode;
			else // phaseType == PhaseType::Single || mPhaseType == PhaseType::A
				return { mSimNode[0] };
		}
		///
		VarType singleVoltage(PhaseType phaseType = PhaseType::Single);
		///
		MatrixVar<VarType> voltage() { return mVoltage; }
		///
		void setSimNode(UInt phase, UInt simNode) { mSimNode[phase] = simNode; }
		///
		void setVoltage(VarType newVoltage) { }

		// #### MNA Section ####
		///
		void mnaUpdateVoltage(Matrix& leftVector);
		///
		void mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVector);
		///
		void mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx);
		/// Return list of MNA tasks
		const Task::List& mnaTasks() {
			return mMnaTasks;
		}
		///
		class MnaPostStepHarm : public Task {
		public:
			MnaPostStepHarm(Node& node, std::vector<Attribute<Matrix>::Ptr> leftVectors) :
				Task(node.mName + ".MnaPostStepHarm"),
				mNode(node), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mNode.attribute("v"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Node& mNode;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};
	};

	namespace SP {
		typedef CPS::Node<Complex> Node;
	}
	namespace DP {
		typedef CPS::Node<Complex> Node;
	}
	namespace EMT {
		typedef CPS::Node<Real> Node;
	}

	template<typename VarType>
	typename Node<VarType>::Ptr Node<VarType>::GND = Node<VarType>::make();

	template<>
	void Node<Real>::mnaUpdateVoltage(Matrix& leftVector);

	template<>
	void Node<Complex>::mnaUpdateVoltage(Matrix& leftVector);

	template<>
	void Node<Complex>::mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVector);

	template<>
	void Node<Complex>::setVoltage(Complex newVoltage);
}
