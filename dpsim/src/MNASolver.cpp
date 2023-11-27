/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/


#include <dpsim/MNASolver.h>
#include <dpsim/SequentialScheduler.h>
#include <memory>

using namespace DPsim;
using namespace CPS;

namespace DPsim {


template <typename VarType>
MnaSolver<VarType>::MnaSolver(String name, CPS::Domain domain, CPS::Logger::Level logLevel) :
	Solver(name, logLevel), mDomain(domain) {

	// Raw source and solution vector logging
	mLeftVectorLog = std::make_shared<DataLogger>(name + "_LeftVector", logLevel == CPS::Logger::Level::trace);
	mRightVectorLog = std::make_shared<DataLogger>(name + "_RightVector", logLevel == CPS::Logger::Level::trace);
}

template <typename VarType>
void MnaSolver<VarType>::setSystem(const CPS::SystemTopology &system) {
	mSystem = system;
}

template <typename VarType>
void MnaSolver<VarType>::initialize() {
	// TODO: check that every system matrix has the same dimensions
	SPDLOG_LOGGER_INFO(mSLog, "---- Start initialization ----");

	// Register attribute for solution vector
	///FIXME: This is kinda ugly... At least we should somehow unify mLeftSideVector and mLeftSideVectorHarm.
	// Best case we have some kind of sub-attributes for attribute vectors / tensor attributes...
	if (mFrequencyParallel) {
		SPDLOG_LOGGER_INFO(mSLog, "Computing network harmonics in parallel.");
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
			mLeftSideVectorHarm.push_back(AttributeStatic<Matrix>::make());
		}
	}
	else {
		mLeftSideVector = AttributeStatic<Matrix>::make();
	}

	SPDLOG_LOGGER_INFO(mSLog, "-- Process topology");
	for (auto comp : mSystem.mComponents)
		SPDLOG_LOGGER_INFO(mSLog, "Added {:s} '{:s}' to simulation.", comp->type(), comp->name());

	// Otherwise LU decomposition will fail
	if (mSystem.mComponents.size() == 0)
		throw SolverException();

	// We need to differentiate between power and signal components and
	// ground nodes should be ignored.
	identifyTopologyObjects();
	// These steps complete the network information.
	collectVirtualNodes();
	assignMatrixNodeIndices();

	SPDLOG_LOGGER_INFO(mSLog, "-- Create empty MNA system matrices and vectors");
	createEmptyVectors();
	createEmptySystemMatrix();

	// Initialize components from powerflow solution and
	// calculate MNA specific initialization values.
	initializeComponents();

	if (mSteadyStateInit) {
		mIsInInitialization = true;
		steadyStateInitialization();
	}
	mIsInInitialization = false;

	// Some components feature a different behaviour for simulation and initialization
	for (auto comp : mSystem.mComponents) {
		auto powerComp = std::dynamic_pointer_cast<CPS::TopologicalPowerComp>(comp);
		if (powerComp) powerComp->setBehaviour(TopologicalPowerComp::Behaviour::MNASimulation);

		auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
		if (sigComp) sigComp->setBehaviour(SimSignalComp::Behaviour::Simulation);
	}

	// Initialize system matrices and source vector.
	initializeSystem();

	SPDLOG_LOGGER_INFO(mSLog, "--- Initialization finished ---");
	SPDLOG_LOGGER_INFO(mSLog, "--- Initial system matrices and vectors ---");
	logSystemMatrices();

	mSLog->flush();
}

template <>
void MnaSolver<Real>::initializeComponents() {
	SPDLOG_LOGGER_INFO(mSLog, "-- Initialize components from power flow");

	CPS::MNAInterface::List allMNAComps;
	allMNAComps.insert(allMNAComps.end(), mMNAComponents.begin(), mMNAComponents.end());
	allMNAComps.insert(allMNAComps.end(), mMNAIntfVariableComps.begin(), mMNAIntfVariableComps.end());

	for (auto comp : allMNAComps) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<Real>>(comp);
		if (!pComp)	continue;
		pComp->checkForUnconnectedTerminals();
		if (mInitFromNodesAndTerminals)
			pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSimSignalComps)
		comp->initialize(mSystem.mSystemOmega, mTimeStep);

	// Initialize MNA specific parts of components.
	for (auto comp : allMNAComps) {
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);
		const Matrix& stamp = comp->getRightVector()->get();
		if (stamp.size() != 0) {
			mRightVectorStamps.push_back(&stamp);
		}
	}

	for (auto comp : mMNAIntfSwitches)
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);
		
	// Initialize nodes
	for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); ++nodeIdx)
		mNodes[nodeIdx]->initialize();
}

template <>
void MnaSolver<Complex>::initializeComponents() {
	SPDLOG_LOGGER_INFO(mSLog, "-- Initialize components from power flow");

	CPS::MNAInterface::List allMNAComps;
	allMNAComps.insert(allMNAComps.end(), mMNAComponents.begin(), mMNAComponents.end());
	allMNAComps.insert(allMNAComps.end(), mMNAIntfVariableComps.begin(), mMNAIntfVariableComps.end());

	// Initialize power components with frequencies and from powerflow results
	for (auto comp : allMNAComps) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp);
		if (!pComp)	continue;
		pComp->checkForUnconnectedTerminals();
		if (mInitFromNodesAndTerminals)
			pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSimSignalComps)
		comp->initialize(mSystem.mSystemOmega, mTimeStep);

	SPDLOG_LOGGER_INFO(mSLog, "-- Initialize MNA properties of components");
	if (mFrequencyParallel) {
		// Initialize MNA specific parts of components.
		for (auto comp : mMNAComponents) {
			// Initialize MNA specific parts of components.
			comp->mnaInitializeHarm(mSystem.mSystemOmega, mTimeStep, mLeftSideVectorHarm);
			const Matrix& stamp = comp->getRightVector()->get();
			if (stamp.size() != 0) mRightVectorStamps.push_back(&stamp);
		}
		// Initialize nodes
		for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); ++nodeIdx) {
			mNodes[nodeIdx]->mnaInitializeHarm(mLeftSideVectorHarm);
		}
	}
	else {
		// Initialize MNA specific parts of components.
		for (auto comp : allMNAComps) {
			comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);
			const Matrix& stamp = comp->getRightVector()->get();
			if (stamp.size() != 0) {
				mRightVectorStamps.push_back(&stamp);
			}
		}

		for (auto comp : mMNAIntfSwitches)
			comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);

		// Initialize nodes
		for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); ++nodeIdx)
			mNodes[nodeIdx]->initialize();
	}
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystem() {
	SPDLOG_LOGGER_INFO(mSLog, "-- Initialize MNA system matrices and source vector");
	mRightSideVector.setZero();

	// just a sanity check in case we change the static
	// initialization of the switch number in the future
	if (mSwitches.size() > sizeof(std::size_t)*8) {
		throw SystemError("Too many Switches.");
	}

	if (mFrequencyParallel)
		initializeSystemWithParallelFrequencies();
	else if (mSystemMatrixRecomputation)
		initializeSystemWithVariableMatrix();
	else
		initializeSystemWithPrecomputedMatrices();
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystemWithParallelFrequencies() {
	// iterate over all possible switch state combinations and frequencies
	for (std::size_t sw = 0; sw < (1ULL << mSwitches.size()); ++sw) {
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
			switchedMatrixEmpty(sw, freq);
			switchedMatrixStamp(sw, freq, mMNAComponents, mSwitches);
		}
	}

	if (mSwitches.size() > 0)
		updateSwitchStatus();

	// Initialize source vector
	for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
		for (auto comp : mMNAComponents)
			comp->mnaApplyRightSideVectorStampHarm(mRightSideVectorHarm[freq], freq);
	}
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystemWithPrecomputedMatrices() {
	// iterate over all possible switch state combinations
	for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
		switchedMatrixEmpty(i);
	}

	if (mSwitches.size() < 1) {
		switchedMatrixStamp(0, mMNAComponents);
	}
	else {
		// Generate switching state dependent system matrices
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
			switchedMatrixStamp(i, mMNAComponents);
		}
		updateSwitchStatus();
	}

	// Initialize source vector for debugging
	// CAUTION: this does not always deliver proper source vector initialization
	// as not full pre-step is executed (not involving necessary electrical or signal
	// subcomp updates before right vector calculation)
	for (auto comp : mMNAComponents) {
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		SPDLOG_LOGGER_DEBUG(mSLog, "Stamping {:s} {:s} into source vector",
			idObj->type(), idObj->name());
		if (mSLog->should_log(spdlog::level::trace))
			mSLog->trace("\n{:s}", Logger::matrixToString(mRightSideVector));
	}
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystemWithVariableMatrix() {

	// Collect index pairs of varying matrix entries from components
	for (auto varElem : mVariableComps)
		for (auto varEntry : varElem->mVariableSystemMatrixEntries)
			mListVariableSystemMatrixEntries.push_back(varEntry);
	SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
	for (auto indexPair : mListVariableSystemMatrixEntries)
		SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);

	stampVariableSystemMatrix();

	// Initialize source vector for debugging
	// CAUTION: this does not always deliver proper source vector initialization
	// as not full pre-step is executed (not involving necessary electrical or signal
	// subcomp updates before right vector calculation)
	for (auto comp : mMNAComponents) {
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		SPDLOG_LOGGER_DEBUG(mSLog, "Stamping {:s} {:s} into source vector",
			idObj->type(), idObj->name());
		if (mSLog->should_log(spdlog::level::trace))
			mSLog->trace("\n{:s}", Logger::matrixToString(mRightSideVector));
	}
}

template <typename VarType>
Bool MnaSolver<VarType>::hasVariableComponentChanged() {
	for (auto varElem : mVariableComps) {
		if (varElem->hasParameterChanged()) {
			auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(varElem);
			SPDLOG_LOGGER_DEBUG(mSLog, "Component ({:s} {:s}) value changed -> Update System Matrix",
				idObj->type(), idObj->name());
			return true;
		}
	}
	return false;
}

template <typename VarType>
void MnaSolver<VarType>::updateSwitchStatus() {
	for (UInt i = 0; i < mSwitches.size(); ++i) {
		mCurrentSwitchStatus.set(i, mSwitches[i]->mnaIsClosed());
	}
}

template <typename VarType>
void MnaSolver<VarType>::identifyTopologyObjects() {
	for (auto baseNode : mSystem.mNodes) {
		// Add nodes to the list and ignore ground nodes.
		if (!baseNode->isGround()) {
			auto node = std::dynamic_pointer_cast< CPS::SimNode<VarType> >(baseNode);
			mNodes.push_back(node);
			SPDLOG_LOGGER_INFO(mSLog, "Added node {:s}", node->name());
		}
	}

	for (auto comp : mSystem.mComponents) {

		auto genComp = std::dynamic_pointer_cast<CPS::MNASyncGenInterface>(comp);
		if (genComp) {
			mSyncGen.push_back(genComp);
		}
		
		auto swComp = std::dynamic_pointer_cast<CPS::MNASwitchInterface>(comp);
		if (swComp) {
			mSwitches.push_back(swComp);
			auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(swComp);
			if (mnaComp) mMNAIntfSwitches.push_back(mnaComp);
		}

		auto varComp = std::dynamic_pointer_cast<CPS::MNAVariableCompInterface>(comp);
		if (varComp) {
			mVariableComps.push_back(varComp);
			auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(varComp);
			if (mnaComp) mMNAIntfVariableComps.push_back(mnaComp);
		}

		if (!(swComp || varComp)) {
			auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(comp);
			if (mnaComp) mMNAComponents.push_back(mnaComp);

			auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
			if (sigComp) mSimSignalComps.push_back(sigComp);
		}
	}
}

template <typename VarType>
void MnaSolver<VarType>::assignMatrixNodeIndices() {
	UInt matrixNodeIndexIdx = 0;
	for (UInt idx = 0; idx < mNodes.size(); ++idx) {
		mNodes[idx]->setMatrixNodeIndex(0, matrixNodeIndexIdx);
		SPDLOG_LOGGER_INFO(mSLog, "Assigned index {} to phase A of node {}", matrixNodeIndexIdx, idx);
		++matrixNodeIndexIdx;
		if (mNodes[idx]->phaseType() == CPS::PhaseType::ABC) {
			mNodes[idx]->setMatrixNodeIndex(1, matrixNodeIndexIdx);
			SPDLOG_LOGGER_INFO(mSLog, "Assigned index {} to phase B of node {}", matrixNodeIndexIdx, idx);
			++matrixNodeIndexIdx;
			mNodes[idx]->setMatrixNodeIndex(2, matrixNodeIndexIdx);
			SPDLOG_LOGGER_INFO(mSLog, "Assigned index {} to phase C of node {}", matrixNodeIndexIdx, idx);
			++matrixNodeIndexIdx;
		}
		// This should be true when the final network node is reached, not considering virtual nodes
		if (idx == mNumNetNodes-1) mNumNetMatrixNodeIndices = matrixNodeIndexIdx;
	}
	// Total number of network nodes including virtual nodes is matrixNodeIndexIdx + 1, which is why the variable is incremented after assignment
	mNumMatrixNodeIndices = matrixNodeIndexIdx;
	mNumVirtualMatrixNodeIndices = mNumMatrixNodeIndices - mNumNetMatrixNodeIndices;
	mNumHarmMatrixNodeIndices = static_cast<UInt>(mSystem.mFrequencies.size()-1) * mNumMatrixNodeIndices;
	mNumTotalMatrixNodeIndices = static_cast<UInt>(mSystem.mFrequencies.size()) * mNumMatrixNodeIndices;

	SPDLOG_LOGGER_INFO(mSLog, "Assigned simulation nodes to topology nodes:");
	SPDLOG_LOGGER_INFO(mSLog, "Number of network simulation nodes: {:d}", mNumNetMatrixNodeIndices);
	SPDLOG_LOGGER_INFO(mSLog, "Number of simulation nodes: {:d}", mNumMatrixNodeIndices);
	SPDLOG_LOGGER_INFO(mSLog, "Number of harmonic simulation nodes: {:d}", mNumHarmMatrixNodeIndices);
}

template<>
void MnaSolver<Real>::createEmptyVectors() {
	mRightSideVector = Matrix::Zero(mNumMatrixNodeIndices, 1);
	**mLeftSideVector = Matrix::Zero(mNumMatrixNodeIndices, 1);
}

template<>
void MnaSolver<Complex>::createEmptyVectors() {
	if (mFrequencyParallel) {
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
			mRightSideVectorHarm.push_back(Matrix::Zero(2*(mNumMatrixNodeIndices), 1));
			mLeftSideVectorHarm.push_back(AttributeStatic<Matrix>::make(Matrix::Zero(2*(mNumMatrixNodeIndices), 1)));
		}
	}
	else {
		mRightSideVector = Matrix::Zero(2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 1);
		**mLeftSideVector = Matrix::Zero(2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 1);
	}
}

template <typename VarType>
void MnaSolver<VarType>::collectVirtualNodes() {
	// We have not added virtual nodes yet so the list has only network nodes
	mNumNetNodes = (UInt) mNodes.size();
	// virtual nodes are placed after network nodes
	UInt virtualNode = mNumNetNodes - 1;

	for (auto comp : mMNAComponents) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
		if (!pComp)	continue;

		// Check if component requires virtual node and if so get a reference
		if (pComp->hasVirtualNodes()) {
			for (UInt node = 0; node < pComp->virtualNodesNumber(); ++node) {
				mNodes.push_back(pComp->virtualNode(node));
				SPDLOG_LOGGER_INFO(mSLog, "Collected virtual node {} of {}", virtualNode, node, pComp->name());
			}
		}

		// Repeat the same steps for virtual nodes of sub components
		// TODO: recursive behavior
		if (pComp->hasSubComponents()) {
			for (auto pSubComp : pComp->subComponents()) {
				for (UInt node = 0; node < pSubComp->virtualNodesNumber(); ++node) {
					mNodes.push_back(pSubComp->virtualNode(node));
					SPDLOG_LOGGER_INFO(mSLog, "Collected virtual node {} of {}", virtualNode, node, pComp->name());
				}
			}
		}
	}

	// collect virtual nodes of variable components
	for (auto comp : mVariableComps) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
		if (!pComp)	continue;

		// Check if component requires virtual node and if so get a reference
		if (pComp->hasVirtualNodes()) {
			for (UInt node = 0; node < pComp->virtualNodesNumber(); ++node) {
				mNodes.push_back(pComp->virtualNode(node));
				SPDLOG_LOGGER_INFO(mSLog, "Collected virtual node {} of Varible Comp {}", node, pComp->name());
			}
		}
	}

	// Update node number to create matrices and vectors
	mNumNodes = (UInt) mNodes.size();
	mNumVirtualNodes = mNumNodes - mNumNetNodes;
	SPDLOG_LOGGER_INFO(mSLog, "Created virtual nodes:");
	SPDLOG_LOGGER_INFO(mSLog, "Number of network nodes: {:d}", mNumNetNodes);
	SPDLOG_LOGGER_INFO(mSLog, "Number of network and virtual nodes: {:d}", mNumNodes);
}

template <typename VarType>
void MnaSolver<VarType>::steadyStateInitialization() {
	SPDLOG_LOGGER_INFO(mSLog, "--- Run steady-state initialization ---");

	DataLogger initLeftVectorLog(mName + "_InitLeftVector", mLogLevel != CPS::Logger::Level::off);
	DataLogger initRightVectorLog(mName + "_InitRightVector", mLogLevel != CPS::Logger::Level::off);

	TopologicalPowerComp::Behaviour initBehaviourPowerComps = TopologicalPowerComp::Behaviour::Initialization;
	SimSignalComp::Behaviour initBehaviourSignalComps = SimSignalComp::Behaviour::Initialization;

	// TODO: enable use of timestep distinct from simulation timestep
	Real initTimeStep = mTimeStep;

	Int timeStepCount = 0;
	Real time = 0;
	Real maxDiff = 1.0;
	Real max = 1.0;
	Matrix diff = Matrix::Zero(2 * mNumNodes, 1);
	Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);

	SPDLOG_LOGGER_INFO(mSLog, "Time step is {:f}s for steady-state initialization", initTimeStep);

	for (auto comp : mSystem.mComponents) {
		auto powerComp = std::dynamic_pointer_cast<CPS::TopologicalPowerComp>(comp);
		if (powerComp) powerComp->setBehaviour(initBehaviourPowerComps);

		auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
		if (sigComp) sigComp->setBehaviour(initBehaviourSignalComps);
	}

	initializeSystem();
	logSystemMatrices();

	// Use sequential scheduler
	SequentialScheduler sched;
	CPS::Task::List tasks;
	Scheduler::Edges inEdges, outEdges;

	for (auto node : mNodes) {
		for (auto task : node->mnaTasks())
			tasks.push_back(task);
	}
	for (auto comp : mMNAComponents) {
		for (auto task : comp->mnaTasks()) {
			tasks.push_back(task);
		}
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : mSimSignalComps) {
		for (auto task : comp->getTasks()) {
			tasks.push_back(task);
		}
	}
	tasks.push_back(createSolveTask());

	sched.resolveDeps(tasks, inEdges, outEdges);
	sched.createSchedule(tasks, inEdges, outEdges);

	while (time < mSteadStIniTimeLimit) {
		// Reset source vector
		mRightSideVector.setZero();

		sched.step(time, timeStepCount);

		if (mDomain == CPS::Domain::EMT) {
			initLeftVectorLog.logEMTNodeValues(time, leftSideVector());
			initRightVectorLog.logEMTNodeValues(time, rightSideVector());
		}
		else {
			initLeftVectorLog.logPhasorNodeValues(time, leftSideVector());
			initRightVectorLog.logPhasorNodeValues(time, rightSideVector());
		}

		// Calculate new simulation time
		time = time + initTimeStep;
		++timeStepCount;

		// Calculate difference
		diff = prevLeftSideVector - **mLeftSideVector;
		prevLeftSideVector = **mLeftSideVector;
		maxDiff = diff.lpNorm<Eigen::Infinity>();
		max = (**mLeftSideVector).lpNorm<Eigen::Infinity>();
		// If difference is smaller than some epsilon, break
		if ((maxDiff / max) < mSteadStIniAccLimit)
			break;
	}

	SPDLOG_LOGGER_INFO(mSLog, "Max difference: {:f} or {:f}% at time {:f}", maxDiff, maxDiff / max, time);

	// Reset system for actual simulation
	mRightSideVector.setZero();

	SPDLOG_LOGGER_INFO(mSLog, "--- Finished steady-state initialization ---");
}

template <typename VarType>
Task::List MnaSolver<VarType>::getTasks() {
	Task::List l;

	for (auto comp : mMNAComponents) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (auto comp : mMNAIntfSwitches) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (auto node : mNodes) {
		for (auto task : node->mnaTasks())
			l.push_back(task);
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : mSimSignalComps) {
		for (auto task : comp->getTasks()) {
			l.push_back(task);
		}
	}
	if (mFrequencyParallel) {
		for (UInt i = 0; i < mSystem.mFrequencies.size(); ++i)
			l.push_back(createSolveTaskHarm(i));
	} else if (mSystemMatrixRecomputation) {
		for (auto comp : this->mMNAIntfVariableComps) {
			for (auto task : comp->mnaTasks())
				l.push_back(task);
		}
		l.push_back(createSolveTaskRecomp());
	} else {
		l.push_back(createSolveTask());
		l.push_back(createLogTask());
	}
	return l;
}


template <typename VarType>
void MnaSolver<VarType>::log(Real time, Int timeStepCount) {
	if (mLogLevel == Logger::Level::off)
		return;

	if (mDomain == CPS::Domain::EMT) {
		mLeftVectorLog->logEMTNodeValues(time, leftSideVector());
		mRightVectorLog->logEMTNodeValues(time, rightSideVector());
	}
	else {
		mLeftVectorLog->logPhasorNodeValues(time, leftSideVector());
		mRightVectorLog->logPhasorNodeValues(time, rightSideVector());
	}
}

}

template class DPsim::MnaSolver<Real>;
template class DPsim::MnaSolver<Complex>;
