/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/


#include <dpsim/MNASolver.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolver<VarType>::MnaSolver(String name, CPS::Domain domain, CPS::Logger::Level logLevel) :
	Solver(name, logLevel), mDomain(domain) {

	// Raw source and solution vector logging
	mLeftVectorLog = std::make_shared<DataLogger>(name + "_LeftVector", logLevel != CPS::Logger::Level::off);
	mRightVectorLog = std::make_shared<DataLogger>(name + "_RightVector", logLevel != CPS::Logger::Level::off);
}

template <typename VarType>
void MnaSolver<VarType>::setSystem(CPS::SystemTopology system) {
	mSystem = system;
}

template <typename VarType>
void MnaSolver<VarType>::initialize() {
	// TODO: check that every system matrix has the same dimensions
	mSLog->info("---- Start initialization ----");

	mSLog->info("-- Process topology");
	for (auto comp : mSystem.mComponents)
		mSLog->info("Added {:s} '{:s}' to simulation.", comp->type(), comp->name());

	// Otherwise LU decomposition will fail
	if (mSystem.mComponents.size() == 0)
		throw SolverException();

	// We need to differentiate between power and signal components and
	// ground nodes should be ignored.
	identifyTopologyObjects();
	// These steps complete the network information.
	collectVirtualNodes();
	assignMatrixNodeIndices();

	mSLog->info("-- Create empty MNA system matrices and vectors");
	createEmptyVectors();
	createEmptySystemMatrix();

	// Register attribute for solution vector
	if (mFrequencyParallel) {
		mSLog->info("Computing network harmonics in parallel.");
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
			addAttribute<Matrix>("left_vector_"+std::to_string(freq), mLeftSideVectorHarm.data()+freq, Flags::read);
			mLeftVectorHarmAttributes.push_back(attribute<Matrix>("left_vector_"+std::to_string(freq)));
		}
	}
	else {
		addAttribute<Matrix>("left_vector", &mLeftSideVector, Flags::read);
	}

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
		if (powerComp) powerComp->setBehaviour(TopologicalPowerComp::Behaviour::Simulation);

		auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
		if (sigComp) sigComp->setBehaviour(SimSignalComp::Behaviour::Simulation);
	}

	// Initialize system matrices and source vector.
	initializeSystem();

	mSLog->info("--- Initialization finished ---");
	mSLog->info("--- Initial system matrices and vectors ---");
	logSystemMatrices();

	mSLog->flush();
}

template <>
void MnaSolver<Real>::initializeComponents() {
	mSLog->info("-- Initialize components from power flow");
	for (auto comp : mMNAComponents) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<Real>>(comp);
		if (!pComp)	continue;
		pComp->checkForUnconnectedTerminals();
		pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSimSignalComps)
		comp->initialize(mSystem.mSystemOmega, mTimeStep);

	// Initialize MNA specific parts of components.
	for (auto comp : mMNAComponents) {
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, attribute<Matrix>("left_vector"));
		const Matrix& stamp = comp->template attribute<Matrix>("right_vector")->get();
		if (stamp.size() != 0) {
			mRightVectorStamps.push_back(&stamp);
		}
	}
	for (auto comp : mSwitches)
		comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, attribute<Matrix>("left_vector"));
}

template <>
void MnaSolver<Complex>::initializeComponents() {
	mSLog->info("-- Initialize components from power flow");

	// Initialize power components with frequencies and from powerflow results
	for (auto comp : mMNAComponents) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp);
		if (!pComp)	continue;
		pComp->checkForUnconnectedTerminals();
		pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSimSignalComps)
		comp->initialize(mSystem.mSystemOmega, mTimeStep);

	mSLog->info("-- Initialize MNA properties of components");
	if (mFrequencyParallel) {
		// Initialize MNA specific parts of components.
		for (auto comp : mMNAComponents) {
			// Initialize MNA specific parts of components.
			comp->mnaInitializeHarm(mSystem.mSystemOmega, mTimeStep, mLeftVectorHarmAttributes);
			const Matrix& stamp = comp->template attribute<Matrix>("right_vector")->get();
			if (stamp.size() != 0) mRightVectorStamps.push_back(&stamp);
		}
		// Initialize nodes
		for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); nodeIdx++) {
			mNodes[nodeIdx]->mnaInitializeHarm(mLeftVectorHarmAttributes);
		}
	}
	else {
		// Initialize MNA specific parts of components.
		for (auto comp : mMNAComponents) {
			comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, attribute<Matrix>("left_vector"));
			const Matrix& stamp = comp->template attribute<Matrix>("right_vector")->get();
			if (stamp.size() != 0) {
				mRightVectorStamps.push_back(&stamp);
			}
		}
		for (auto comp : mSwitches)
			comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, attribute<Matrix>("left_vector"));
	}
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystem() {
	mSLog->info("-- Initialize MNA system matrices and source vector");
	mRightSideVector.setZero();

	// just a sanity check in case we change the static
	// initialization of the switch number in the future
	if (mSwitches.size() > sizeof(std::size_t)*8) {
		throw SystemError("Too many Switches.");
	}

	if (mFrequencyParallel)
		initializeSystemWithParallelFrequencies();
	else
		initializeSystemWithPrecomputedMatrices();
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystemWithParallelFrequencies() {
	// iterate over all possible switch state combinations
	for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++)
			mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(i)][freq].setZero();
	}

	for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
		// Create system matrix if no switches were added
		// TODO add case for switches and possibly merge with no harmonics
		for (auto comp : mMNAComponents)
			comp->mnaApplySystemMatrixStampHarm(mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(0)][freq], freq);

		mLuFactorizationsHarm[std::bitset<SWITCH_NUM>(0)].push_back(
			Eigen::PartialPivLU<Matrix>(mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(0)][freq]));

		// Initialize source vector
		for (auto comp : mMNAComponents)
			comp->mnaApplyRightSideVectorStampHarm(mRightSideVectorHarm[freq], freq);
	}
}

template <typename VarType>
void MnaSolver<VarType>::initializeSystemWithPrecomputedMatrices() {
	// iterate over all possible switch state combinations
	for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)].setZero();
	}

	if (mSwitches.size() < 1) {
		// Create system matrix if no switches were added
		for (auto comp : mMNAComponents) {
			comp->mnaApplySystemMatrixStamp(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
			auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
			mSLog->debug("Stamping {:s} {:s} into system matrix",
				idObj->type(), idObj->name());
			if (mSLog->should_log(spdlog::level::trace)) {
				mSLog->trace("\n{:s}",
					Logger::matrixToString(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]));
			}
		}
#ifdef WITH_SPARSE
		mLuFactorizations[std::bitset<SWITCH_NUM>(0)].analyzePattern(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
		mLuFactorizations[std::bitset<SWITCH_NUM>(0)].factorize(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
#else
		mLuFactorizations[std::bitset<SWITCH_NUM>(0)] = Eigen::PartialPivLU<Matrix>(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
#endif
	}
	else {
		// Generate switching state dependent system matrices
		for (auto& sys : mSwitchedMatrices) {
			for (auto comp : mMNAComponents)
				comp->mnaApplySystemMatrixStamp(sys.second);
			for (UInt i = 0; i < mSwitches.size(); i++)
				mSwitches[i]->mnaApplySwitchSystemMatrixStamp(sys.second, sys.first[i]);
			// Compute LU-factorization for system matrix
#ifdef WITH_SPARSE
			mLuFactorizations[sys.first].analyzePattern(sys.second);
			mLuFactorizations[sys.first].factorize(sys.second);
#else
			mLuFactorizations[sys.first] = Eigen::PartialPivLU<Matrix>(sys.second);
#endif
		}
		updateSwitchStatus();
	}

	// Initialize source vector for debugging
	for (auto comp : mMNAComponents) {
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		mSLog->debug("Stamping {:s} {:s} into source vector",
			idObj->type(), idObj->name());
		if (mSLog->should_log(spdlog::level::trace))
			mSLog->trace("\n{:s}", Logger::matrixToString(mRightSideVector));
	}
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
			auto node = std::dynamic_pointer_cast< CPS::SimNode<VarType> >(baseNode);
			mNodes.push_back(node);
			mSLog->info("Added node {:s}", node->name());
		}
	}

	for (auto comp : mSystem.mComponents) {

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
	for (UInt idx = 0; idx < mNodes.size(); idx++) {
		mNodes[idx]->setMatrixNodeIndex(0, matrixNodeIndexIdx);
		mSLog->info("Assigned index {} to phase A of node {}", matrixNodeIndexIdx, idx);
		matrixNodeIndexIdx++;
		if (mNodes[idx]->phaseType() == CPS::PhaseType::ABC) {
			mNodes[idx]->setMatrixNodeIndex(1, matrixNodeIndexIdx);
			mSLog->info("Assigned index {} to phase B of node {}", matrixNodeIndexIdx, idx);
			matrixNodeIndexIdx++;
			mNodes[idx]->setMatrixNodeIndex(2, matrixNodeIndexIdx);
			mSLog->info("Assigned index {} to phase B of node {}", matrixNodeIndexIdx, idx);
			matrixNodeIndexIdx++;
		}
		if (idx == mNumNetNodes-1) mNumNetMatrixNodeIndices = matrixNodeIndexIdx;
	}
	// Total number of network nodes is matrixNodeIndexIdx + 1
	mNumMatrixNodeIndices = matrixNodeIndexIdx;
	mNumVirtualMatrixNodeIndices = mNumMatrixNodeIndices - mNumNetMatrixNodeIndices;
	mNumHarmMatrixNodeIndices = static_cast<UInt>(mSystem.mFrequencies.size()-1) * mNumMatrixNodeIndices;

	mSLog->info("Assigned simulation nodes to topology nodes:");
	mSLog->info("Number of network simulation nodes: {:d}", mNumNetMatrixNodeIndices);
	mSLog->info("Number of simulation nodes: {:d}", mNumMatrixNodeIndices);
	mSLog->info("Number of harmonic simulation nodes: {:d}", mNumHarmMatrixNodeIndices);
}

template<>
void MnaSolver<Real>::createEmptyVectors() {
	mRightSideVector = Matrix::Zero(mNumMatrixNodeIndices, 1);
	mLeftSideVector = Matrix::Zero(mNumMatrixNodeIndices, 1);
}

template<>
void MnaSolver<Complex>::createEmptyVectors() {
	if (mFrequencyParallel) {
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
			mRightSideVectorHarm.push_back(Matrix::Zero(2*(mNumMatrixNodeIndices), 1));
			mLeftSideVectorHarm.push_back(Matrix::Zero(2*(mNumMatrixNodeIndices), 1));
		}
	}
	else {
		mRightSideVector = Matrix::Zero(2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 1);
		mLeftSideVector = Matrix::Zero(2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 1);
	}
}

template<>
void MnaSolver<Real>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

#ifdef WITH_SPARSE
	for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++)
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)].resize(mNumMatrixNodeIndices, mNumMatrixNodeIndices);

	mBaseSystemMatrix.resize(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
#else
	for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++)
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(mNumMatrixNodeIndices, mNumMatrixNodeIndices);

	mBaseSystemMatrix = Matrix::Zero(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
#endif
}

template<>
void MnaSolver<Complex>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mFrequencyParallel) {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); i++) {
			for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
				mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(i)].push_back(
					Matrix::Zero(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices)));
			}
		}
	}
	else {
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
#ifdef WITH_SPARSE
			mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)].resize(2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices));
#else
			mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 2*(mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices));
#endif
		}
	}
#ifdef WITH_SPARSE
	mBaseSystemMatrix.resize(2 * (mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 2 * (mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices));
#else
	mBaseSystemMatrix = Matrix::Zero(2 * (mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 2 * (mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices));
#endif
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
			for (UInt node = 0; node < pComp->virtualNodesNumber(); node++) {
				mNodes.push_back(pComp->virtualNode(node));
				mSLog->info("Collected virtual node {} of {}", virtualNode, node, pComp->name());
			}
		}

		// Repeat the same steps for virtual nodes of sub components
		// TODO: recursive behavior
		if (pComp->hasSubComponents()) {
			for (auto pSubComp : pComp->subComponents()) {
				for (UInt node = 0; node < pSubComp->virtualNodesNumber(); node++) {
					mNodes.push_back(pSubComp->virtualNode(node));
					mSLog->info("Collected virtual node {} of {}", virtualNode, node, pComp->name());
				}
			}
		}
	}

	// Update node number to create matrices and vectors
	mNumNodes = (UInt) mNodes.size();
	mNumVirtualNodes = mNumNodes - mNumNetNodes;
	mSLog->info("Created virtual nodes:");
	mSLog->info("Number of network nodes: {:d}", mNumNetNodes);
	mSLog->info("Number of network and virtual nodes: {:d}", mNumNodes);
}

template <typename VarType>
void MnaSolver<VarType>::steadyStateInitialization() {
	mSLog->info("--- Run steady-state initialization ---");

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

	mSLog->info("Time step is {:f}s for steady-state initialization", initTimeStep);

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
	tasks.push_back(std::make_shared<MnaSolver<VarType>::SolveTask>(*this));

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
		timeStepCount++;

		// Calculate difference
		diff = prevLeftSideVector - mLeftSideVector;
		prevLeftSideVector = mLeftSideVector;
		maxDiff = diff.lpNorm<Eigen::Infinity>();
		max = mLeftSideVector.lpNorm<Eigen::Infinity>();
		// If difference is smaller than some epsilon, break
		if ((maxDiff / max) < mSteadStIniAccLimit)
			break;
	}

	mSLog->info("Max difference: {:f} or {:f}% at time {:f}", maxDiff, maxDiff / max, time);

	// Reset system for actual simulation
	mRightSideVector.setZero();

	mSLog->info("--- Finished steady-state initialization ---");
}

template <typename VarType>
Task::List MnaSolver<VarType>::getTasks() {
	Task::List l;

	for (auto comp : mMNAComponents) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (auto comp : mSwitches) {
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
		for (UInt i = 0; i < mSystem.mFrequencies.size(); i++)
			l.push_back(std::make_shared<MnaSolver<VarType>::SolveTaskHarm>(*this, i));
	} else {
		l.push_back(std::make_shared<MnaSolver<VarType>::SolveTask>(*this));
		l.push_back(std::make_shared<MnaSolver<VarType>::LogTask>(*this));
	}
	return l;
}

template <typename VarType>
void MnaSolver<VarType>::solve(Real time, Int timeStepCount) {
	// Reset source vector
	mRightSideVector.setZero();

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVector += *stamp;

	if (mSwitchedMatrices.size() > 0)
		mLeftSideVector = mLuFactorizations[mCurrentSwitchStatus].solve(mRightSideVector);

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; nodeIdx++)
		mNodes[nodeIdx]->mnaUpdateVoltage(mLeftSideVector);

	if (!mIsInInitialization)
		updateSwitchStatus();

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolver<VarType>::solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) {
	mRightSideVectorHarm[freqIdx].setZero();

	// Sum of right side vectors (computed by the components' pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

	mLeftSideVectorHarm[freqIdx] =	mLuFactorizationsHarm[mCurrentSwitchStatus][freqIdx].solve(mRightSideVectorHarm[freqIdx]);
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

template <typename VarType>
void MnaSolver<VarType>::logSystemMatrices() {
	if (mFrequencyParallel) {
		for (UInt i = 0; i < mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(0)].size(); i++) {
			mSLog->info("System matrix for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(0)][i]));
			//mSLog->info("LU decomposition for frequency: {:d} \n{:s}", i,
			//	Logger::matrixToString(mLuFactorizationsHarm[std::bitset<SWITCH_NUM>(0)][i].matrixLU()));
		}

		for (UInt i = 0; i < mRightSideVectorHarm.size(); i++)
			mSLog->info("Right side vector for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mRightSideVectorHarm[i]));

	}
	else {
		if (mSwitches.size() < 1) {
			mSLog->info("System matrix: \n{}", mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
			//mSLog->info("LU decomposition: \n{}",	mLuFactorizations[std::bitset<SWITCH_NUM>(0)].matrixLU());
		}
		else {
			mSLog->info("Initial switch status: {:s}", mCurrentSwitchStatus.to_string());

			for (auto sys : mSwitchedMatrices) {
				mSLog->info("Switching System matrix {:s} \n{:s}",
					sys.first.to_string(), Logger::matrixToString(sys.second));
				//mSLog->info("LU Factorization for System Matrix {:s} \n{:s}",
				//	sys.first.to_string(), Logger::matrixToString(mLuFactorizations[sys.first].matrixLU()));
			}
		}
		mSLog->info("Right side vector: \n{}", mRightSideVector);
	}
}

}

template class DPsim::MnaSolver<Real>;
template class DPsim::MnaSolver<Complex>;
