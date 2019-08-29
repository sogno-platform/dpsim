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
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolver<VarType>::MnaSolver(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
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
	mSLog->info("---- Start initialization ----");

	mSLog->info("-- Process system components");
	for (auto comp : mSystem.mComponents)
		mSLog->info("Added {:s} '{:s}' to simulation.", comp->type(), comp->name());

	// Otherwise LU decomposition will fail
	if (mSystem.mComponents.size() == 0)
		throw SolverException();

	// We need to differentiate between power and signal components and
	// ground nodes should be ignored.
	identifyTopologyObjects();
	// These steps complete the network information.
	createVirtualNodes();
	assignSimNodes();

	mSLog->info("-- Create empty MNA system matrices and vectors");
	// The system topology is prepared and we create the MNA matrices.
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

	// This steady state initialization is MNA specific and runs a simulation
	// before the actual simulation executed by the user.
	if (mSteadyStateInit && mDomain == CPS::Domain::DP)
		steadyStateInitialization();

	// Some components feature a different behaviour for simulation and initialization
	for (auto comp : mSystem.mComponents)
		comp->setBehaviour(Component::Behaviour::Simulation);

	// Initialize system matrices and source vector.
	initializeSystem();

	mSLog->info("--- Initialization finished ---");
	mSLog->info("--- Initial system matrices and vectors ---");
	logSystemMatrices();
}

template <>
void MnaSolver<Real>::initializeComponents() {
	mSLog->info("-- Initialize components from power flow");
	for (auto comp : mPowerComponents) {
		auto pComp = std::dynamic_pointer_cast<PowerComponent<Real>>(comp);
		if (!pComp)	continue;
		pComp->initializeFromPowerflow(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSignalComponents)
		comp->initialize(mSystem.mSystemOmega, mTimeStep);

	// Initialize MNA specific parts of components.
	for (auto comp : mPowerComponents) {
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
	for (auto comp : mPowerComponents) {
		auto pComp = std::dynamic_pointer_cast<PowerComponent<Complex>>(comp);
		if (!pComp)	continue;
		pComp->initializeFromPowerflow(mSystem.mSystemFrequency);
	}

	// Initialize signal components.
	for (auto comp : mSignalComponents)
		comp->initialize(mSystem.mSystemOmega, mTimeStep);

	mSLog->info("-- Initialize MNA properties of components");
	if (mFrequencyParallel) {
		// Initialize MNA specific parts of components.
		for (auto comp : mPowerComponents) {
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
		for (auto comp : mPowerComponents) {
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

	if (mFrequencyParallel) {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); i++) {
			for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++)
				mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(i)][freq].setZero();
		}
	}
	else {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); i++)
			mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)].setZero();
	}

	if (mFrequencyParallel) {
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
			// Create system matrix if no switches were added
			for (auto comp : mPowerComponents)
				comp->mnaApplySystemMatrixStampHarm(mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(0)][freq], freq);

			mLuFactorizationsHarm[std::bitset<SWITCH_NUM>(0)].push_back(
				Eigen::PartialPivLU<Matrix>(mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(0)][freq]));

			// Initialize source vector
			for (auto comp : mPowerComponents)
				comp->mnaApplyRightSideVectorStampHarm(mRightSideVectorHarm[freq], freq);
		}
	}
	else {
		if (mSwitches.size() < 1) {
			// Create system matrix if no switches were added
			for (auto comp : mPowerComponents) {
				comp->mnaApplySystemMatrixStamp(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
				auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
				mSLog->debug("Stamping {:s} {:s} into system matrix: \n{:s}",
					idObj->type(), idObj->name(), Logger::matrixToString(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]));
			}
			mLuFactorizations[std::bitset<SWITCH_NUM>(0)] = Eigen::PartialPivLU<Matrix>(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]);
		}
		else {
			// Generate switching state dependent system matrices
			for (auto& sys : mSwitchedMatrices) {
				for (auto comp : mPowerComponents)
					comp->mnaApplySystemMatrixStamp(sys.second);
				for (UInt i = 0; i < mSwitches.size(); i++)
					mSwitches[i]->mnaApplySwitchSystemMatrixStamp(sys.second, sys.first[i]);
				// Compute LU-factorization for system matrix
				mLuFactorizations[sys.first] = Eigen::PartialPivLU<Matrix>(sys.second);
			}
			updateSwitchStatus();
		}
		// Initialize source vector for debugging
		for (auto comp : mPowerComponents) {
			comp->mnaApplyRightSideVectorStamp(mRightSideVector);
			auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);

			mSLog->debug("Stamping {:s} {:s} into source vector: \n{:s}",
				idObj->type(), idObj->name(), Logger::matrixCompToString(mRightSideVector));
		}
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
			auto node = std::dynamic_pointer_cast< CPS::Node<VarType> >(baseNode);
			mNodes.push_back( node );
		}
	}

	for (UInt i = 0; i < mNodes.size(); i++)
		mSLog->info("Added node {:s}", mNodes[i]->name());;

	for (auto comp : mSystem.mComponents) {
		auto swComp = std::dynamic_pointer_cast<CPS::MNASwitchInterface>(comp);
		if (swComp) {
			mSwitches.push_back(swComp);
			continue;
		}

		auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(comp);
		if (mnaComp) mPowerComponents.push_back(mnaComp);

		auto sigComp = std::dynamic_pointer_cast<CPS::SignalComponent>(comp);
		if (sigComp) mSignalComponents.push_back(sigComp);
	}
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
	mNumHarmSimNodes = (mSystem.mFrequencies.size()-1) * mNumSimNodes;

	mSLog->info("Assigned simulation nodes to topology nodes:");
	mSLog->info("Number of network simulation nodes: {:d}", mNumNetSimNodes);
	mSLog->info("Number of simulation nodes: {:d}", mNumSimNodes);
	mSLog->info("Number of harmonic simulation nodes: {:d}", mNumHarmSimNodes);
}

template<>
void MnaSolver<Real>::createEmptyVectors() {
	mRightSideVector = Matrix::Zero(mNumSimNodes, 1);
	mLeftSideVector = Matrix::Zero(mNumSimNodes, 1);
}

template<>
void MnaSolver<Complex>::createEmptyVectors() {
	if (mFrequencyParallel) {
		for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
			mRightSideVectorHarm.push_back(Matrix::Zero(2*(mNumSimNodes), 1));
			mLeftSideVectorHarm.push_back(Matrix::Zero(2*(mNumSimNodes), 1));
		}
	}
	else {
		mRightSideVector = Matrix::Zero(2*(mNumSimNodes + mNumHarmSimNodes), 1);
		mLeftSideVector = Matrix::Zero(2*(mNumSimNodes + mNumHarmSimNodes), 1);
	}
}

template<>
void MnaSolver<Real>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	for (UInt i = 0; i < std::pow(2,mSwitches.size()); i++)
		mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(mNumSimNodes, mNumSimNodes);
}

template<>
void MnaSolver<Complex>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mFrequencyParallel) {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); i++) {
			for(Int freq = 0; freq < mSystem.mFrequencies.size(); freq++) {
				mSwitchedMatricesHarm[std::bitset<SWITCH_NUM>(i)].push_back(
					Matrix::Zero(2*(mNumSimNodes), 2*(mNumSimNodes)));
			}
		}
	}
	else {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); i++)
			mSwitchedMatrices[std::bitset<SWITCH_NUM>(i)] = Matrix::Zero(2*(mNumSimNodes + mNumHarmSimNodes), 2*(mNumSimNodes + mNumHarmSimNodes));
	}
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
				mNodes.push_back(pComp->virtualNode(node));

				pComp->virtualNode(node)->setSimNode(0, virtualNode);
				mSLog->info("Assigned index {} to virtual node {} for {}", virtualNode, node, pComp->name());

				if (pComp->virtualNode(node)->phaseType() == CPS::PhaseType::ABC) {
					for (UInt phase = 1; phase < 3; phase++) {
						virtualNode++;
						pComp->virtualNode(node)->setSimNode(phase, virtualNode);
						mSLog->info("Assigned index {} to virtual node {} for {}", virtualNode, node, pComp->name());
					}
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

	Int timeStepCount = 0;
	Real time = 0;
	Real eps = 0.0001;
	Real maxDiff = 0, max = 0;
	Matrix diff;
	Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);

	for (auto comp : mSystem.mComponents)
		comp->setBehaviour(Component::Behaviour::Initialization);

	initializeSystem();
	logSystemMatrices();

	updateSwitchStatus();

	// Use sequential scheduler
	SequentialScheduler sched;
	CPS::Task::List tasks;
	Scheduler::Edges inEdges, outEdges;

	for (auto node : mNodes) {
		for (auto task : node->mnaTasks())
			tasks.push_back(task);
	}
	for (auto comp : mPowerComponents) {
		for (auto task : comp->mnaTasks()) {
			tasks.push_back(task);
		}
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : mSignalComponents) {
		for (auto task : comp->getTasks()) {
			tasks.push_back(task);
		}
	}
	tasks.push_back(std::make_shared<MnaSolver<VarType>::SolveTask>(*this, true));

	sched.resolveDeps(tasks, inEdges, outEdges);
	sched.createSchedule(tasks, inEdges, outEdges);

	while (time < 10) {
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
		time = time + mTimeStep;
		timeStepCount++;

		// Calculate difference
		diff = prevLeftSideVector - mLeftSideVector;
		prevLeftSideVector = mLeftSideVector;
		maxDiff = diff.lpNorm<Eigen::Infinity>();
		max = mLeftSideVector.lpNorm<Eigen::Infinity>();
		// If difference is smaller than some epsilon, break
		if ((maxDiff / max) < eps)
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

	for (auto comp : mPowerComponents) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (auto node : mNodes) {
		for (auto task : node->mnaTasks())
			l.push_back(task);
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : mSignalComponents) {
		for (auto task : comp->getTasks()) {
			l.push_back(task);
		}
	}
	if (mFrequencyParallel) {
		for (UInt i = 0; i < mSystem.mFrequencies.size(); i++)
			l.push_back(std::make_shared<MnaSolver<VarType>::SolveTaskHarm>(*this, false, i));
	} else {
		l.push_back(std::make_shared<MnaSolver<VarType>::SolveTask>(*this, false));
		l.push_back(std::make_shared<MnaSolver<VarType>::LogTask>(*this));
	}
	return l;
}

template <typename VarType>
void MnaSolver<VarType>::SolveTask::execute(Real time, Int timeStepCount) {
	// Reset source vector
	mSolver.mRightSideVector.setZero();

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mSolver.mRightVectorStamps)
		mSolver.mRightSideVector += *stamp;

	if (mSolver.mSwitchedMatrices.size() > 0)
		mSolver.mLeftSideVector = mSolver.mLuFactorizations[mSolver.mCurrentSwitchStatus].solve(mSolver.mRightSideVector);

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mSolver.mNumNetNodes; nodeIdx++)
		mSolver.mNodes[nodeIdx]->mnaUpdateVoltage(mSolver.mLeftSideVector);

	if (!mSteadyStateInit)
		mSolver.updateSwitchStatus();

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolver<VarType>::SolveTaskHarm::execute(Real time, Int timeStepCount) {
	mSolver.mRightSideVectorHarm[mFreqIdx].setZero();

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mSolver.mRightVectorStamps)
		mSolver.mRightSideVectorHarm[mFreqIdx] += stamp->col(mFreqIdx);

	mSolver.mLeftSideVectorHarm[mFreqIdx] =
		mSolver.mLuFactorizationsHarm[mSolver.mCurrentSwitchStatus][mFreqIdx].solve(mSolver.mRightSideVectorHarm[mFreqIdx]);
}

template <typename VarType>
void MnaSolver<VarType>::log(Real time) {
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
			mSLog->info("LU decomposition for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mLuFactorizationsHarm[std::bitset<SWITCH_NUM>(0)][i].matrixLU()));
		}

		for (UInt i = 0; i < mRightSideVectorHarm.size(); i++)
			mSLog->info("Right side vector for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mRightSideVectorHarm[i]));

	}
	else {
		if (mSwitches.size() < 1) {
			mSLog->info("System matrix: \n{:s}",
				Logger::matrixToString(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)]));
			mSLog->info("LU decomposition: \n{:s}",
				Logger::matrixToString(mLuFactorizations[std::bitset<SWITCH_NUM>(0)].matrixLU()));
		}
		else {
			mSLog->info("Initial switch status: {:s}", mCurrentSwitchStatus.to_string());

			for (auto sys : mSwitchedMatrices) {
				mSLog->info("Switching System matrix {:s} \n{:s}",
					sys.first.to_string(), Logger::matrixToString(sys.second));
				mSLog->info("LU Factorization for System Matrix {:s} \n{:s}",
					sys.first.to_string(), Logger::matrixToString(mLuFactorizations[sys.first].matrixLU()));
			}
		}
		mSLog->info("Right side vector: \n{:s}", Logger::matrixToString(mRightSideVector));
	}
}

template <typename VarType>
void MnaSolver<VarType>::LogTask::execute(Real time, Int timeStepCount) {
	mSolver.log(time);
}

}

template class DPsim::MnaSolver<Real>;
template class DPsim::MnaSolver<Complex>;
