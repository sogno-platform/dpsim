/*********************************************************************************
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

#include <dpsim/MNASolver.h>

using namespace DPsim;
using namespace CPS;

template <typename VarType>
void MnaSolver<VarType>::initialize(CPS::SystemTopology system) {
	mLog.info() << "#### Start Initialization ####" << std::endl;
	mSystem = system;

	// We need to differentiate between power and signal components and
	// ground nodes should be ignored.
	IdentifyTopologyObjects();

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
	for (auto comp : mPowerComponents)
		comp->initializeFromPowerflow(mSystem.mSystemFrequency);

	// This steady state initialization is MNA specific and runs a simulation 
	// before the actual simulation executed by the user.
	if (mSteadyStateInit && mDomain == CPS::Domain::DP) {
		mLog.info() << "Run steady-state initialization." << std::endl;
		steadyStateInitialization();
	}

	// Now, initialize the components for the actual simulation.			
	for (auto comp : mSignalComponents) {
		comp->initialize();
	}

	// Create initial system matrix
	for (auto comp : mPowerComponents) {
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);		
		comp->mnaApplySystemMatrixStamp(mSystemMatrix);		
	}

	for (auto comp : mSwitches) {
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);		
		comp->mnaApplySystemMatrixStamp(mSystemMatrix);		
	}
	
	// Compute LU-factorization for system matrix
	mLuFactorization = Eigen::PartialPivLU<Matrix>(mSystemMatrix);

	// Generate switching state dependent matrix
	for (auto& sys : mSwitchedMatrices) {		
		for (auto comp : mPowerComponents) {
			comp->mnaApplySystemMatrixStamp(sys.second);		
		}	
		for (UInt i = 0; i < mSwitches.size(); i++) {			
			mSwitches[i]->mnaApplySwitchSystemMatrixStamp(sys.second, sys.first[i]);		
		}
		mLuFactorizations[sys.first] = Eigen::PartialPivLU<Matrix>(sys.second);
	}

	updateSwitchStatus();

	// Logging
	for (auto comp : system.mComponents)
		mLog.info() << "Added " << comp->type() << " '" << comp->name() << "' to simulation." << std::endl;

	mLog.info() << "System matrix:" << std::endl;
	mLog.info(mSystemMatrix);
	mLog.info() << "LU decomposition:" << std::endl;
	mLog.info(mLuFactorization.matrixLU());
	mLog.info() << "Right side vector:" << std::endl;
	mLog.info(mRightSideVector);

	for (auto sys : mSwitchedMatrices) {   
		mLog.info() << "Switching System matrix " << sys.first << std::endl;
		mLog.info(sys.second);
		mLog.info(mLuFactorizations[sys.first].matrixLU());
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
void MnaSolver<VarType>::IdentifyTopologyObjects() {			
	for (auto baseNode : mSystem.mNodes) {	
		// Add nodes to the list and ignore ground nodes.
		if (!baseNode->isGround()) {			
			auto node = std::dynamic_pointer_cast< CPS::Node<VarType> >(baseNode);
			mNodes.push_back( node );	
		}
	}
	
	for (UInt i = 0; i < mNodes.size(); i++) {
		mLog.info() << "Found node " << mNodes[i]->name() << std::endl;
	}

	for (auto comp : mSystem.mComponents) {
		if ( CPS::MNASwitchInterface::Ptr Switch = std::dynamic_pointer_cast<CPS::MNASwitchInterface>(comp) ) {
			mSwitches.push_back(Switch);
		} 
		// TODO: cast to MNAInterface instead		
		else if ( typename CPS::PowerComponent<VarType>::Ptr powercomp = std::dynamic_pointer_cast< CPS::PowerComponent<VarType> >(comp) ) {
			mPowerComponents.push_back(powercomp);		
		}
		else if ( CPS::SignalComponent::Ptr signalcomp = std::dynamic_pointer_cast< CPS::SignalComponent >(comp) )	{	
			mSignalComponents.push_back(signalcomp);	
		}
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
		if (mNodes[idx]->getPhaseType() == CPS::PhaseType::ABC) {
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
		mLeftSideVector = mLuFactorization.solve(mRightSideVector);
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
	mSystemMatrix = Matrix::Zero(mNumSimNodes, mNumSimNodes);
	
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	for (UInt i = 0; i < mSwitches.size() * 2; i++) {
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(mNumSimNodes, mNumSimNodes);
	}
}

template<>
void MnaSolver<Complex>::createEmptySystemMatrix() {
	mSystemMatrix = Matrix::Zero(2 * mNumSimNodes, 2 * mNumSimNodes);		

	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	for (UInt i = 0; i < mSwitches.size() * 2; i++) {
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(2 * mNumSimNodes, 2 * mNumSimNodes);
	}	
}

template <typename VarType>
void MnaSolver<VarType>::createVirtualNodes() {
	// We have not added virtual nodes yet so the list has only network nodes
	mNumNetNodes = mNodes.size();
	// virtual nodes are placed after network nodes
	UInt virtualNode = mNumNetNodes - 1;
	// Check if component requires virtual node and if so set one
	for (auto comp : mPowerComponents) {
		if (comp->hasVirtualNodes()) {
			for (UInt node = 0; node < comp->virtualNodesNumber(); node++) {
				virtualNode++;
				mNodes.push_back(std::make_shared<CPS::Node<VarType>>(virtualNode));
				comp->setVirtualNodeAt(mNodes[virtualNode], node);
				mLog.info() << "Created virtual node" << node << " = " << virtualNode
					<< " for " << comp->name() << std::endl;
			}
		}
	}
	// Update node number to create matrices and vectors
	mNumNodes = mNodes.size();
	mNumVirtualNodes = mNumNodes - mNumNetNodes;

	mLog.info() << "Number of network nodes: " << mNumNetNodes << std::endl;
	mLog.info() << "Number of nodes: " << mNumNodes << std::endl;
}

template <typename VarType>
void MnaSolver<VarType>::steadyStateInitialization() {
	Real time = 0;

	// Initialize right side vector and components
	for (auto comp : mPowerComponents) {
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		comp->mnaApplySystemMatrixStamp(mSystemMatrix);
		comp->mnaApplyInitialSystemMatrixStamp(mSystemMatrix);
	}
	// Compute LU-factorization for system matrix
	mLuFactorization = Eigen::PartialPivLU<Matrix>(mSystemMatrix);

	Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
	Matrix diff;
	Real maxDiff, max;

	while (time < 10) {
		time = step(time);

		diff = prevLeftSideVector - mLeftSideVector;
		prevLeftSideVector = mLeftSideVector;
		maxDiff = diff.lpNorm<Eigen::Infinity>();
		max = mLeftSideVector.lpNorm<Eigen::Infinity>();

		if ((maxDiff / max) < 0.0001)
			break;
	}
	mLog.info() << "Initialization finished. Max difference: "
		<< maxDiff << " or " << maxDiff / max << "% at time " << time << std::endl;

	createEmptySystemMatrix();
}

template <typename VarType>
Real MnaSolver<VarType>::step(Real time) {
	mRightSideVector.setZero();

	// First, step signal components and then power components
	for (auto comp : mSignalComponents) {
		comp->step(time);
	}
	for (auto comp : mPowerComponents) {
		comp->mnaStep(mSystemMatrix, mRightSideVector, mLeftSideVector, time);
	}
	for (auto comp : mSwitches) {
	comp->mnaStep(mSystemMatrix, mRightSideVector, mLeftSideVector, time);
	}
	
	// Solve MNA system
	solve();

	// Some components need to update internal states
	for (auto comp : mPowerComponents) {
		comp->mnaPostStep(mRightSideVector, mLeftSideVector, time);
	}

	// TODO Try to avoid this step.
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; nodeIdx++) {
		mNodes[nodeIdx]->mnaUpdateVoltage(mLeftSideVector);
	}

	updateSwitchStatus();
	mLog.debug() << "Switch status is " << mCurrentSwitchStatus << " for " << time << std::endl;

	// Calculate new simulation time
	return time + mTimeStep;
}

template class DPsim::MnaSolver<Real>;
template class DPsim::MnaSolver<Complex>;
