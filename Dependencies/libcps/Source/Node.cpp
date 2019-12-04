/** Power System Node
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
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

#include <cps/Node.h>

using namespace CPS;

template <typename VarType>
Node<VarType>::Node(String name, String uid,
	std::vector<UInt> simNode, PhaseType phaseType, std::vector<Complex> initialVoltage)
	: TopologicalNode(uid, name, phaseType, initialVoltage) {

	if (phaseType == PhaseType::ABC) {
		mSimNode = simNode;
		mVoltage = MatrixVar<VarType>::Zero(3, 1);
	}
	else {
		mSimNode[0] = simNode[0];
		mVoltage = MatrixVar<VarType>::Zero(1, 1);
	}

	addAttribute<MatrixVar<VarType>>("v", &mVoltage, Flags::read);
}

template <typename VarType>
Node<VarType>::Node(PhaseType phaseType)
	: Node("gnd", "gnd", { 0, 0, 0 }, phaseType, { 0, 0, 0 }) {
		mIsGround = true;
		mInitialVoltage = MatrixComp::Zero(3, 1);
		mVoltage = MatrixVar<VarType>::Zero(3, 1);
}

template <typename VarType>
void Node<VarType>::initialize(Matrix frequencies) {
	mFrequencies = frequencies;
	mNumFreqs = static_cast<UInt>(mFrequencies.size());
	Matrix::Index rowNum = phaseType() == PhaseType::ABC ? 3 : 1;
	mVoltage = MatrixVar<VarType>::Zero(rowNum, mNumFreqs);
}

template <typename VarType>
VarType Node<VarType>::singleVoltage(PhaseType phaseType) {
	if (phaseType == PhaseType::B)
		return mVoltage(1,0);
	else if (phaseType == PhaseType::C)
		return mVoltage(2,0);
	else // phaseType == PhaseType::Single || mPhaseType == PhaseType::A
		return mVoltage(0,0);
}

template<>
void Node<Complex>::setVoltage(Complex newVoltage) {
	mVoltage(0, 0) = newVoltage;
}

template<>
void Node<Real>::mnaUpdateVoltage(Matrix& leftVector) {
	if (mSimNode[0] >= 0) mVoltage(0,0) = Math::realFromVectorElement(leftVector, mSimNode[0]);
	if (mPhaseType == PhaseType::ABC) {
		if (mSimNode[1] >= 0) mVoltage(1,0) = Math::realFromVectorElement(leftVector, mSimNode[1]);
		if (mSimNode[2] >= 0) mVoltage(2,0) = Math::realFromVectorElement(leftVector, mSimNode[2]);
	}
}

template<>
void Node<Complex>::mnaUpdateVoltage(Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
			if (mSimNode[0] >= 0) mVoltage(0,freq) = Math::complexFromVectorElement(leftVector, mSimNode[0], mNumFreqs, freq);
		if (mPhaseType == PhaseType::ABC) {
			if (mSimNode[1] >= 0) mVoltage(1,freq) = Math::complexFromVectorElement(leftVector, mSimNode[1], mNumFreqs, freq);
			if (mSimNode[2] >= 0) mVoltage(2,freq) = Math::complexFromVectorElement(leftVector, mSimNode[2], mNumFreqs, freq);
		}
	}
}

template<>
void Node<Real>::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) { }

template<>
void Node<Complex>::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) {
	if (mSimNode[0] >= 0) mVoltage(0,freqIdx) = Math::complexFromVectorElement(leftVector, mSimNode[0]);
	if (mPhaseType == PhaseType::ABC) {
		if (mSimNode[1] >= 0) mVoltage(1,freqIdx) = Math::complexFromVectorElement(leftVector, mSimNode[1]);
		if (mSimNode[2] >= 0) mVoltage(2,freqIdx) = Math::complexFromVectorElement(leftVector, mSimNode[2]);
	}
}

template <>
void Node<Real>::mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVectors) { }

template <>
void Node<Complex>::mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	mMnaTasks = {
		std::make_shared<MnaPostStepHarm>(*this, leftVectors)
	};
}

template <>
void Node<Complex>::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	for (UInt freq = 0; freq < mNode.mNumFreqs; freq++)
		mNode.mnaUpdateVoltageHarm(*mLeftVectors[freq], freq);
}

template <>
void Node<Real>::MnaPostStepHarm::execute(Real time, Int timeStepCount) {

}

template class CPS::Node<Real>;
template class CPS::Node<Complex>;
