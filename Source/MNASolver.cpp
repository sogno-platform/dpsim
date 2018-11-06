/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/MNASolver.h>

using namespace DPsim;
using namespace CPS;

template <typename VarType>
void MnaSolver<VarType>::initialize(CPS::SystemTopology system) {
	mLog.info() << "#### Start Initialization ####" << std::endl;
	mSystem = system;

	if (mSystem.mComponents.size() == 0)
		throw SolverException(); // Otherwise LU decomposition will fail

	// We need to differentiate between power and signal components and
	// ground nodes should be ignored.
	identifyTopologyObjects();

	// These steps complete the network information.
	createVirtualNodes();
	assignSimNodes();

	// For the power components the step order should not be important
	// but signal components need to be executed following the connections.
	sortExecutionPriority();

	// The system topology is prepared and we create the MNA matrices.
	createEmptyVectors();
	createEmptySystemMatrix();

	// TODO: Move to base solver class?
	// This intialization according to power flow information is not MNA specific.
	mLog.info() << "Initialize power flow" << std::endl;
	for (auto comp : mPowerComponents) {
		auto pComp = std::dynamic_pointer_cast<PowerComponent<VarType>>(comp);
		if (!pComp)	continue;
		pComp->initializeFromPowerflow(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSignalComponents)
		comp->initialize();
	// Initialize MNA specific parts of components.
	for (auto comp : mPowerComponents)
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);
	for (auto comp : mSwitches)
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);

	// This steady state initialization is MNA specific and runs a simulation
	// before the actual simulation executed by the user.
	if (mSteadyStateInit && mDomain == CPS::Domain::DP) {
		mLog.info() << "Run steady-state initialization." << std::endl;
		steadyStateInitialization();
		mLog.info() << "Finished steady-state initialization." << std::endl;
	}

	for (auto comp : mSystem.mComponents)
		comp->setBehaviour(Component::Behaviour::Simulation);

	// Create initial system matrix
	for (auto comp : mPowerComponents) {
		comp->mnaApplySystemMatrixStamp(mTmpSystemMatrix);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		mLog.debug() << "Stamping " << idObj->type() << " " << idObj->name()
					<< " into system matrix: \n" << mTmpSystemMatrix << std::endl;
	}
	for (auto comp : mSwitches) {
		comp->mnaApplySystemMatrixStamp(mTmpSystemMatrix);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		mLog.debug() << "Stamping " << idObj->type() << " " << idObj->name()
			<< " into system matrix: \n" << mTmpSystemMatrix << std::endl;
	}

	// Compute LU-factorization for system matrix
	mTmpLuFactorization = Eigen::PartialPivLU<Matrix>(mTmpSystemMatrix);

	// Generate switching state dependent matrix
	for (auto& sys : mSwitchedMatrices) {
		for (auto comp : mPowerComponents)
			comp->mnaApplySystemMatrixStamp(sys.second);
		for (UInt i = 0; i < mSwitches.size(); i++)
			mSwitches[i]->mnaApplySwitchSystemMatrixStamp(sys.second, sys.first[i]);

		mLuFactorizations[sys.first] = Eigen::PartialPivLU<Matrix>(sys.second);
	}
	updateSwitchStatus();

	// Initialize source vector for debugging
	for (auto comp : mPowerComponents) {
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		mLog.debug() << "Stamping " << idObj->type() << " " << idObj->name()
			<< " into source vector: \n" << mRightSideVector << std::endl;
	}

	// Logging
	for (auto comp : system.mComponents)
		mLog.info() << "Added " << comp->type() << " '" << comp->name() << "' to simulation." << std::endl;

	mLog.info() << "System matrix: \n" << mTmpSystemMatrix << std::endl;
	mLog.info() << "LU decomposition: \n" << mTmpLuFactorization.matrixLU() << std::endl;
	mLog.info() << "Right side vector: \n" << mRightSideVector << std::endl;

	for (auto sys : mSwitchedMatrices) {
		mLog.info() << "Switching System matrix "
					<< sys.first << ": \n" << sys.second << std::endl;
		mLog.info() << "LU Factorization for System Matrix "
					<< sys.first << ": \n" << mLuFactorizations[sys.first].matrixLU() << std::endl;
	}

	mLog.info() << "Initial switch status: " << mCurrentSwitchStatus << std::endl;
}

template <typename VarType>
void MnaSolver<VarType>::updateSwitchStatus() {
	for (UInt i = 0; i < mSwitches.size(); i++) {
		mCurrentSwitchStatus.set(i, mSwitches[i]->mnaIsClosed());
	}
}

template <typename VarType>
void MnaSolver<VarType>::identifyTopologyObjects() {
	for (auto baseNode : mSystem.mNodes) {
		// Add nodes to the list and ignore ground nodes.
		if (!baseNode->isGround()) {
			auto node = std::dynamic_pointer_cast< CPS::Node<VarType> >(baseNode);
			mNodes.push_back( node );
		}
	}

	for (UInt i = 0; i < mNodes.size(); i++)
		mLog.info() << "Found node " << mNodes[i]->name() << std::endl;

	for (auto comp : mSystem.mComponents) {
		auto swComp = std::dynamic_pointer_cast<CPS::MNASwitchInterface>(comp);
		if (swComp) mSwitches.push_back(swComp);

		auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(comp);
		if (mnaComp) mPowerComponents.push_back(mnaComp);

		auto sigComp = std::dynamic_pointer_cast<CPS::SignalComponent>(comp);
		if (sigComp) mSignalComponents.push_back(sigComp);
	}
}

template <typename VarType>
void MnaSolver<VarType>::sortExecutionPriority() {
	// Sort SignalComponents according to execution priority.
	// Components with a higher priority number should come first.
	std::sort(mSignalComponents.begin(), mSignalComponents.end(), [] (const auto& lhs, const auto& rhs) -> bool {
		return lhs->priority() > rhs->priority();
	});
}

template <typename VarType>
void MnaSolver<VarType>::assignSimNodes() {
	UInt simNodeIdx = 0;
	for (UInt idx = 0; idx < mNodes.size(); idx++) {
		mNodes[idx]->setSimNode(0, simNodeIdx);
		simNodeIdx++;
		if (mNodes[idx]->phaseType() == CPS::PhaseType::ABC) {
			mNodes[idx]->setSimNode(1, simNodeIdx);
			simNodeIdx++;
			mNodes[idx]->setSimNode(2, simNodeIdx);
			simNodeIdx++;
		}
		if (idx == mNumNetNodes-1) mNumNetSimNodes = simNodeIdx;
	}
	// Total number of network nodes is simNodeIdx + 1
	mNumSimNodes = simNodeIdx;
	mNumVirtualSimNodes = mNumSimNodes - mNumNetSimNodes;

	mLog.info() << "Number of network simulation nodes: " << mNumNetSimNodes << std::endl;
	mLog.info() << "Number of simulation nodes: " << mNumSimNodes << std::endl;
}

template <typename VarType>
void MnaSolver<VarType>::solve()  {
	if (mSwitchedMatrices.size() > 0)
		mLeftSideVector = mLuFactorizations[mCurrentSwitchStatus].solve(mRightSideVector);
	else
		mLeftSideVector = mTmpLuFactorization.solve(mRightSideVector);
}

template<>
void MnaSolver<Real>::createEmptyVectors() {
	mRightSideVector = Matrix::Zero(mNumSimNodes, 1);
	mLeftSideVector = Matrix::Zero(mNumSimNodes, 1);
}

template<>
void MnaSolver<Complex>::createEmptyVectors() {
	mRightSideVector = Matrix::Zero(2 * mNumSimNodes, 1);
	mLeftSideVector = Matrix::Zero(2 * mNumSimNodes, 1);
}

template<>
void MnaSolver<Real>::createEmptySystemMatrix() {
	mTmpSystemMatrix = Matrix::Zero(mNumSimNodes, mNumSimNodes);

	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	for (UInt i = 0; i < mSwitches.size() * 2; i++)
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(mNumSimNodes, mNumSimNodes);
}

template<>
void MnaSolver<Complex>::createEmptySystemMatrix() {
	mTmpSystemMatrix = Matrix::Zero(2 * mNumSimNodes, 2 * mNumSimNodes);

	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	for (UInt i = 0; i < mSwitches.size() * 2; i++)
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(2 * mNumSimNodes, 2 * mNumSimNodes);
}

template <typename VarType>
void MnaSolver<VarType>::createVirtualNodes() {
	// We have not added virtual nodes yet so the list has only network nodes
	mNumNetNodes = (UInt) mNodes.size();
	// virtual nodes are placed after network nodes
	UInt virtualNode = mNumNetNodes - 1;
	// Check if component requires virtual node and if so set one
	for (auto comp : mPowerComponents) {
		auto pComp = std::dynamic_pointer_cast<PowerComponent<VarType>>(comp);
		if (!pComp)	continue;

		if (pComp->hasVirtualNodes()) {
			for (UInt node = 0; node < pComp->virtualNodesNumber(); node++) {
				virtualNode++;
				mNodes.push_back(std::make_shared<CPS::Node<VarType>>(virtualNode));
				pComp->setVirtualNodeAt(mNodes[virtualNode], node);

				mLog.info() << "Created virtual node" << node << " = " << virtualNode
					<< " for " << pComp->name() << std::endl;
			}
		}
	}
	// Update node number to create matrices and vectors
	mNumNodes = (UInt) mNodes.size();
	mNumVirtualNodes = mNumNodes - mNumNetNodes;

	mLog.info() << "Number of network nodes: " << mNumNetNodes << std::endl;
	mLog.info() << "Number of nodes: " << mNumNodes << std::endl;
}

template <typename VarType>
void MnaSolver<VarType>::steadyStateInitialization() {
	Real time = 0;
	Real eps = 0.0001;
	Real maxDiff, max;
	Matrix diff;
	Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);

	for (auto comp : mSystem.mComponents)
		comp->setBehaviour(Component::Behaviour::Initialization);

	// Create system matrix
	for (auto comp : mPowerComponents)
		comp->mnaApplySystemMatrixStamp(mTmpSystemMatrix);
	for (auto comp : mSwitches)
		comp->mnaApplySystemMatrixStamp(mTmpSystemMatrix);

	// Compute LU-factorization for system matrix
	mTmpLuFactorization = Eigen::PartialPivLU<Matrix>(mTmpSystemMatrix);

	while (time < 10) {
		// Reset source vector
		mRightSideVector.setZero();

		// First, step signal components and then power components
		for (auto comp : mSignalComponents)
			comp->step(time);
		for (auto comp : mPowerComponents)
			comp->mnaStep(mTmpSystemMatrix, mRightSideVector, mLeftSideVector, time);

		// Solve MNA system
		mLeftSideVector = mTmpLuFactorization.solve(mRightSideVector);

		// Some components need to update internal states
		for (auto comp : mPowerComponents)
			comp->mnaPostStep(mRightSideVector, mLeftSideVector, time);

		// TODO Try to avoid this step.
		for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; nodeIdx++)
			mNodes[nodeIdx]->mnaUpdateVoltage(mLeftSideVector);

		if (mDomain == CPS::Domain::EMT) {
			mInitLeftVectorLog.logEMTNodeValues(time, leftSideVector());
			mInitRightVectorLog.logEMTNodeValues(time, rightSideVector());
		}
		else {
			mInitLeftVectorLog.logPhasorNodeValues(time, leftSideVector());
			mInitRightVectorLog.logPhasorNodeValues(time, rightSideVector());
		}

		// Calculate new simulation time
		time = time + mTimeStep;

		// Calculate difference
		diff = prevLeftSideVector - mLeftSideVector;
		prevLeftSideVector = mLeftSideVector;
		maxDiff = diff.lpNorm<Eigen::Infinity>();
		max = mLeftSideVector.lpNorm<Eigen::Infinity>();
		// If difference is smaller than some epsilon, break
		if ((maxDiff / max) < eps)
			break;
	}

	mLog.info() << "Max difference: " << maxDiff << " or "
		<< maxDiff / max << "% at time " << time << std::endl;

	// Reset system for actual simulation
	mTmpSystemMatrix.setZero();
	mRightSideVector.setZero();
}

template <typename VarType>
Real MnaSolver<VarType>::step(Real time) {
	// Reset source vector
	mRightSideVector.setZero();

	// First, step signal components and then power components
	for (auto comp : mSignalComponents)
		comp->step(time);
	for (auto comp : mPowerComponents)
		comp->mnaStep(mTmpSystemMatrix, mRightSideVector, mLeftSideVector, time);
	for (auto comp : mSwitches)
		comp->mnaStep(mTmpSystemMatrix, mRightSideVector, mLeftSideVector, time);

	// Solve MNA system
	solve();

	// Some components need to update internal states
	for (auto comp : mPowerComponents)
		comp->mnaPostStep(mRightSideVector, mLeftSideVector, time);

	// TODO Try to avoid this step.
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; nodeIdx++)
		mNodes[nodeIdx]->mnaUpdateVoltage(mLeftSideVector);

	updateSwitchStatus();
	mLog.debug() << "Switch status is " << mCurrentSwitchStatus << " for " << time << std::endl;

	// Calculate new simulation time
	return time + mTimeStep;
}

template class DPsim::MnaSolver<Real>;
template class DPsim::MnaSolver<Complex>;
