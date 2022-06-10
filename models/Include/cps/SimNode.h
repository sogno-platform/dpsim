/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/TopologicalNode.h>
#include <cps/Task.h>

namespace CPS {

	template <typename VarType>
	class SimNode : public TopologicalNode,
					public std::enable_shared_from_this<SimNode<VarType>>,
					public SharedFactory<SimNode<VarType>> {
	protected:
		///
		std::vector<UInt> mMatrixNodeIndex = { 0 };
		/// List of considered network harmonics
		Matrix mFrequencies;
		/// Number of harmonics
		UInt mNumFreqs = 0;
		///
		Task::List mMnaTasks;
	public:
		typedef VarType Type;
		typedef std::shared_ptr<SimNode<VarType>> Ptr;
		typedef std::vector<Ptr> List;
		///
		static Ptr GND;

		///
		const typename Attribute<MatrixVar<VarType>>::Ptr mVoltage;
		/// Power injected at node
		const typename Attribute<MatrixVar<VarType>>::Ptr mApparentPower;

		/// This very general constructor is used by other constructors.
		SimNode(String uid, String name, std::vector<UInt> matrixNodeIndex,
			PhaseType phaseType, const std::vector<Complex> &initialVoltage);
		/// Create ground node if no parameters are given.
		SimNode(PhaseType phaseType = PhaseType::Single);
		/// Create named node and optionally assigns an initial voltage.
		/// This should be the constructor called by users in examples.
		SimNode(String name, PhaseType phaseType = PhaseType::Single,
			const std::vector<Complex> &initialVoltage = { 0, 0, 0 })
			: SimNode(name, name, { 0, 0, 0 }, phaseType, initialVoltage) { }
		/// Create node with name and node number.
		/// This is mostly used by functions.
		SimNode(String uid, String name, UInt matrixNodeIndex,
			PhaseType phaseType = PhaseType::Single, const std::vector<Complex> &initialVoltage = { 0, 0, 0 })
			: SimNode(uid, name, { matrixNodeIndex, matrixNodeIndex + 1, matrixNodeIndex + 2 }, phaseType, initialVoltage) {}
		/// Create node with default name and node number.
		/// This is mostly used by functions.
		SimNode(UInt matrixNodeIndex, PhaseType phaseType = PhaseType::Single)
			: SimNode("N" + std::to_string(matrixNodeIndex), "N" + std::to_string(matrixNodeIndex), matrixNodeIndex, phaseType) { }

		/// Initialize state matrices with size according to phase type and frequency number
		void initialize(Matrix frequencies);
		/// Returns matrix index for specified phase
		UInt matrixNodeIndex(PhaseType phaseType = PhaseType::Single) {
			if ((phaseType == PhaseType::A || phaseType == PhaseType::Single)
				&& (mPhaseType == PhaseType::Single
				|| mPhaseType == PhaseType::A
				|| mPhaseType == PhaseType::ABC))
				return mMatrixNodeIndex[0];
			else if (phaseType == PhaseType::B
				&& (mPhaseType == PhaseType::B
				|| mPhaseType == PhaseType::ABC))
				return mMatrixNodeIndex[1];
			else if (phaseType == PhaseType::C
				&& (mPhaseType == PhaseType::C
				|| mPhaseType == PhaseType::ABC))
				return mMatrixNodeIndex[2];
			else
				return 0;
		}
		/// Returns all matrix indices
		std::vector<UInt> matrixNodeIndices() {
			if (mPhaseType == PhaseType::B)
				return { mMatrixNodeIndex[1] };
			else if (mPhaseType == PhaseType::C)
				return { mMatrixNodeIndex[2] };
			else if (mPhaseType == PhaseType::ABC)
				return mMatrixNodeIndex;
			else // phaseType == PhaseType::Single || mPhaseType == PhaseType::A
				return { mMatrixNodeIndex[0] };
		}
		///
		VarType singleVoltage(PhaseType phaseType = PhaseType::Single);
		///
		MatrixVar<VarType> voltage() { return **mVoltage; }
		///
		void setMatrixNodeIndex(UInt phase, UInt matrixNodeIndex) { mMatrixNodeIndex[phase] = matrixNodeIndex; }
		///
		void setVoltage(VarType newVoltage) { }
		///
		void setPower(VarType newPower) { }

		// #### MNA Section ####
		///
		void mnaUpdateVoltage(const Matrix& leftVector);
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
			MnaPostStepHarm(SimNode& node, const std::vector<Attribute<Matrix>::Ptr> &leftVectors) :
				Task(**node.mName + ".MnaPostStepHarm"),
				mNode(node), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mNode.attribute("v"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			SimNode& mNode;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};
	};

	namespace SP {
		typedef CPS::SimNode<Complex> SimNode;
	}
	namespace DP {
		typedef CPS::SimNode<Complex> SimNode;
	}
	namespace EMT {
		typedef CPS::SimNode<Real> SimNode;
	}

	template<typename VarType>
	typename SimNode<VarType>::Ptr SimNode<VarType>::GND = SimNode<VarType>::make();

	template<>
	void SimNode<Real>::mnaUpdateVoltage(const Matrix& leftVector);

	template<>
	void SimNode<Complex>::mnaUpdateVoltage(const Matrix& leftVector);

	template<>
	void SimNode<Complex>::mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVector);

	template<>
	void SimNode<Complex>::setVoltage(Complex newVoltage);
}
