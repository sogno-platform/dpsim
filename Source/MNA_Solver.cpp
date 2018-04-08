/** Simulation
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "MNA_Solver.h"

#ifdef WITH_CIM
#include "cps/Source/CIM/Reader.h"
#endif /* WITH_CIM */

using namespace CPS;
using namespace DPsim;

MnaSimulation::MnaSimulation(String name,
	Real timeStep, Real finalTime, SimulationType simType,
	Logger::Level logLevel, Bool steadyStateInit, Int downSampleRate) :
	mLog("Logs/" + name + ".log", logLevel),
	mLeftVectorLog("Logs/" + name + "_LeftVector.csv", logLevel),
	mRightVectorLog("Logs/" + name + "_RightVector.csv", logLevel) {

	mName = name;
	mTimeStep = timeStep;
	mFinalTime = finalTime;
	mSimType = simType;
	mLogLevel = logLevel;
	mDownSampleRate = downSampleRate;
	mSteadyStateInit = steadyStateInit;
}

MnaSimulation::MnaSimulation(String name, SystemTopology system,
	Real timeStep, Real finalTime, SimulationType simType,
	Logger::Level logLevel, Int downSampleRate)
	: MnaSimulation(name, timeStep, finalTime, simType,
		logLevel, false, downSampleRate) {

	initialize(system);

	// Logging
	for (auto comp : system.mComponents)
		mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;

	mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemMatrices[mSystemIndex]);
	mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mLuFactorizations[mSystemIndex].matrixLU());
	mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mRightSideVector);
}

#ifdef WITH_CIM
MnaSimulation::MnaSimulation(String name, std::list<String> cimFiles, Real frequency,
	Real timeStep, Real finalTime, SimulationType simType,
	Logger::Level logLevel, Int downSampleRate)
	: MnaSimulation(name, timeStep, finalTime, simType,
		logLevel, true, downSampleRate) {

	CIM::Reader reader(frequency, logLevel, logLevel);
	reader.addFiles(cimFiles);
	reader.parseFiles();
	SystemTopology system = reader.getSystemTopology();
	initialize(system);

	// Logging
	for (auto comp : system.mComponents)
		mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;

	mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemMatrices[mSystemIndex]);
	mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mLuFactorizations[mSystemIndex].matrixLU());
	mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mRightSideVector);
}
#endif

void MnaSimulation::initialize(SystemTopology system) {
	mSystemTopologies.push_back(system);

	mLog.Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;
	// Calculate the mNumber of nodes by going through the list of components
	// TODO we use the values from the first component vector right now and assume that
	// these values don't change on switches
	Int maxNode = 0;
	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		// determine maximum node in component list
		if (comp->getNode1() > maxNode)
			maxNode = comp->getNode1();
		if (comp->getNode2() > maxNode)
			maxNode = comp->getNode2();
	}

	if (mSystemTopologies[mSystemIndex].mNodes.size() == 0) {
		// Create Nodes for all node indices
		mSystemTopologies[mSystemIndex].mNodes.resize(maxNode + 1, nullptr);
		for (int node = 0; node <= mSystemTopologies[mSystemIndex].mNodes.size(); node++)
			mSystemTopologies[mSystemIndex].mNodes[node] = std::make_shared<Node>(node);

		for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
			std::shared_ptr<Node> node1, node2;
			if (comp->getNode1() < 0)
				node1 = mSystemTopologies[mSystemIndex].mGnd;
			else
				node1 = mSystemTopologies[mSystemIndex].mNodes[comp->getNode1()];
			if (comp->getNode2() < 0)
				node2 = mSystemTopologies[mSystemIndex].mGnd;
			else
				node2 = mSystemTopologies[mSystemIndex].mNodes[comp->getNode2()];

			comp->setNodes(Node::List{ node1, node2 });
		}
	}

	mLog.Log(Logger::Level::INFO) << "Maximum node number: " << maxNode << std::endl;
	// virtual nodes are placed after network nodes
	UInt virtualNode = maxNode;
	mNumRealNodes = maxNode + 1;

	// Check if component requires virtual node and if so set one
	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		if (comp->hasVirtualNodes()) {
			for (Int node = 0; node < comp->getVirtualNodesNum(); node++) {
				virtualNode++;
				mSystemTopologies[mSystemIndex].mNodes.push_back(std::make_shared<Node>(virtualNode));
				comp->setVirtualNodeAt(mSystemTopologies[mSystemIndex].mNodes[virtualNode], node);
				mLog.Log(Logger::Level::INFO) << "Created virtual node" << node << "= " << virtualNode
					<< " for " << comp->getName() << std::endl;
			}
		}
	}

	// Calculate system size and create matrices and vectors
	mNumNodes = virtualNode + 1;
	mNumVirtualNodes = mNumNodes - mNumRealNodes;

	createEmptyVectors();
	createEmptySystemMatrix();

	mLog.Log(Logger::Level::INFO) << "Initialize power flow" << std::endl;
	for (auto comp : mSystemTopologies[mSystemIndex].mComponents)
		comp->initializePowerflow(mSystemTopologies[mSystemIndex].mSystemFrequency);

	if (mSteadyStateInit && mSimType == SimulationType::DP) {
		// Initialize right side vector and components
		for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
			comp->mnaInitialize(mSystemTopologies[mSystemIndex].mSystemOmega, mTimeStep);
			comp->mnaApplyRightSideVectorStamp(mRightSideVector);
			comp->mnaApplySystemMatrixStamp(mSystemMatrices[mSystemIndex]);
			comp->mnaApplyInitialSystemMatrixStamp(mSystemMatrices[mSystemIndex]);
		}
		// Compute LU-factorization for system matrix
		mLuFactorizations.push_back(Eigen::PartialPivLU<Matrix>(mSystemMatrices[mSystemIndex]));

		mLog.Log(Logger::Level::INFO) << "Start initialization." << std::endl;
		Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
		Matrix diff;
		Real maxDiff, max;
		while(mTime < 10) {
			step();
			increaseByTimeStep();
			diff = prevLeftSideVector - mLeftSideVector;
			prevLeftSideVector = mLeftSideVector;
			maxDiff = diff.lpNorm<Eigen::Infinity>();
			max = mLeftSideVector.lpNorm<Eigen::Infinity>();
			if ((maxDiff / max) < 0.0001) break;
		}
		mLog.Log(Logger::Level::INFO) << "Initialization finished. Max difference: "
			<< maxDiff << " or " << maxDiff / max << "% at time " << mTime << std::endl;
		mSystemMatrices.pop_back();
		mLuFactorizations.pop_back();
		mSystemIndex = 0;
		mTime = 0;
		createEmptySystemMatrix();
	}

	// Initialize right side vector and components
	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		comp->mnaInitialize(mSystemTopologies[mSystemIndex].mSystemOmega, mTimeStep);
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		comp->mnaApplySystemMatrixStamp(mSystemMatrices[mSystemIndex]);
	}
	// Compute LU-factorization for system matrix
	mLuFactorizations.push_back(Eigen::PartialPivLU<Matrix>(mSystemMatrices[mSystemIndex]));

}

void MnaSimulation::createEmptyVectors() {
	if (mSimType == SimulationType::EMT) {
		mRightSideVector = Matrix::Zero(mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(mNumNodes, 1);
	}
	else {
		mRightSideVector = Matrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
	}
}

void MnaSimulation::createEmptySystemMatrix() {
	if (mSimType == SimulationType::EMT) {
		mSystemMatrices.push_back(Matrix::Zero(mNumNodes, mNumNodes));
	}
	else {
		mSystemMatrices.push_back(Matrix::Zero(2 * mNumNodes, 2 * mNumNodes));
	}
}

void MnaSimulation::addSystemTopology(SystemTopology system) {
	mSystemTopologies.push_back(system);

	// It is assumed that the system size does not change
	createEmptySystemMatrix();
	for (auto comp : system.mComponents)
		comp->mnaApplySystemMatrixStamp(mSystemMatrices[mSystemMatrices.size()]);

	mLuFactorizations.push_back(LUFactorized(mSystemMatrices[mSystemMatrices.size()]));
}

void MnaSimulation::switchSystemMatrix(Int systemIndex) {
	if (systemIndex < mSystemMatrices.size())
		mSystemIndex = systemIndex;
}

void MnaSimulation::solve()  {
	mLeftSideVector = mLuFactorizations[mSystemIndex].solve(mRightSideVector);
}

void MnaSimulation::step(bool blocking) {
	mRightSideVector.setZero();

	for (auto eif : mExternalInterfaces) {
		eif->readValues(blocking);
	}

	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		comp->mnaStep(mSystemMatrices[mSystemIndex], mRightSideVector, mLeftSideVector, mTime);
	}

	solve();

	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		comp->mnaPostStep(mRightSideVector, mLeftSideVector, mTime);
	}

	for (UInt nodeIdx = 0; nodeIdx < mNumRealNodes; nodeIdx++) {
		mSystemTopologies[mSystemIndex].mNodes[nodeIdx]->mnaUpdateVoltages(mLeftSideVector);
	}

	for (auto eif : mExternalInterfaces) {
		eif->writeValues();
	}

	if (mSwitchTimeIndex < mSwitchEvents.size()) {
		if (mTime >= mSwitchEvents[mSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEvents[mSwitchTimeIndex].systemIndex);
			++mSwitchTimeIndex;

			mLog.Log(Logger::Level::INFO) << "Switched to system " << mSwitchTimeIndex << " at " << mTime << std::endl;
			mLog.Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemMatrices[mSystemIndex] << std::endl;
			mLog.Log(Logger::Level::INFO) << "New decomp:" << std::endl << mLuFactorizations[mSystemIndex].matrixLU() << std::endl;
		}
	}
}

void MnaSimulation::run() {
	mLog.Log(Logger::Level::INFO) << "Start simulation." << std::endl;

	while (mTime < mFinalTime) {
		step();
		mLeftVectorLog.LogNodeValues(getTime(), getLeftSideVector());
		mRightVectorLog.LogNodeValues(getTime(), getRightSideVector());
		increaseByTimeStep();
	}

	mLog.Log(Logger::Level::INFO) << "Simulation finished." << std::endl;
}

void MnaSimulation::run(double duration) {
	mLog.Log(Logger::Level::INFO) << "Run simulation for " << duration << " seconds." << std::endl;
	double started = mTime;

	while ((mTime - started) < duration) {
		step();
		mLeftVectorLog.LogNodeValues(getTime(), getLeftSideVector());
		mRightVectorLog.LogNodeValues(getTime(), getRightSideVector());
		increaseByTimeStep();
	}
	mLog.Log(Logger::Level::INFO) << "Simulation finished." << std::endl;
}

void MnaSimulation::setSwitchTime(Real switchTime, Int systemIndex) {
	SwitchConfiguration newSwitchConf;
	newSwitchConf.switchTime = switchTime;
	newSwitchConf.systemIndex = systemIndex;
	mSwitchEvents.push_back(newSwitchConf);
}
