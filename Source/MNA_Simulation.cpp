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

#include "MNA_Simulation.h"

#ifdef WITH_CIM
#include "CPowerSystems/Source/CIM/Reader.h"
#endif /* WITH_CIM */

using namespace DPsim;

MnaSimulation::MnaSimulation(String name,
	Component::List comps,
	Real frequency, Real timeStep, Real finalTime, SimulationType simType,
	Logger::Level logLevel, Int downSampleRate) :
	mLog("Logs/" + name + ".log", logLevel),
	mLeftVectorLog("Logs/" + name + "_LeftVector.csv", logLevel),
	mRightVectorLog("Logs/" + name + "_RightVector.csv", logLevel) {

	mGnd = std::make_shared<Node>(-1);
	mName = name;
	mTimeStep = timeStep;
	mFinalTime = finalTime;
	mSystemFrequency = frequency;
	mSystemOmega = 2 * PI*frequency;
	mSimType = simType;
	mLogLevel = logLevel;
	mDownSampleRate = downSampleRate;

	initialize(comps);

	// Logging
	for (auto comp : comps)
		mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;

	mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemMatrices[mSystemIndex]);
	mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mLuFactorizations[mSystemIndex].matrixLU());
	mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mRightSideVector);
}

#ifdef WITH_CIM
MnaSimulation::MnaSimulation(String name,
	std::list<String> cimFiles,
	Real frequency, Real timeStep, Real finalTime, SimulationType simType,
	Logger::Level logLevel, Int downSampleRate) :
	mLog("Logs/" + name + ".log", logLevel),
	mLeftVectorLog("Logs/" + name + "_LeftVector.csv", logLevel),
	mRightVectorLog("Logs/" + name + "_RightVector.csv", logLevel) {

	mGnd = std::make_shared<Node>(-1);
	mName = name;
	mTimeStep = timeStep;
	mFinalTime = finalTime;
	mSystemFrequency = frequency;
	mSystemOmega = 2*PI*frequency;
	mSimType = simType;
	mLogLevel = logLevel;
	mDownSampleRate = downSampleRate;

	CIM::Reader reader(frequency, logLevel, logLevel);

	for (String filename : cimFiles) {
		if (!reader.addFile(filename))
			std::cout << "Failed to read file " << filename << std::endl;
	}
	try {
		reader.parseFiles();
	}
	catch (...) {
		std::cerr << "Failed to parse CIM files" << std::endl;
		return;
	}

	mNodes.push_back(reader.getNodes());
	Component::List comps = reader.getComponents();
	initialize(comps);

	// Logging
	for (auto comp : comps)
		mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;

	mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemMatrices[mSystemIndex]);
	mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mLuFactorizations[mSystemIndex].matrixLU());
	mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mRightSideVector);
}
#endif

void MnaSimulation::initialize(Component::List newComponents) {
	mComponents.push_back(newComponents);

	mLog.Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;
	// Calculate the mNumber of nodes by going through the list of components
	// TODO we use the values from the first component vector right now and assume that
	// these values don't change on switches
	Int maxNode = 0;
	for (auto comp : mComponents[mSystemIndex]) {
		// determine maximum node in component list
		if (comp->getNode1() > maxNode)
			maxNode = comp->getNode1();
		if (comp->getNode2() > maxNode)
			maxNode = comp->getNode2();
	}

	if (mNodes[mSystemIndex].size() == 0) {
		// Create Nodes for all node indices
		mNodes.push_back(Node::List(maxNode + 1, nullptr));
		for (int node = 0; node < mNodes.size(); node++)
			mNodes[mSystemIndex][node] = std::make_shared<Node>(node);

		for (auto comp : mComponents[mSystemIndex]) {
			std::shared_ptr<Node> node1, node2;
			if (comp->getNode1() < 0)
				node1 = mGnd;
			else
				node1 = mNodes[mSystemIndex][comp->getNode1()];
			if (comp->getNode2() < 0)
				node2 = mGnd;
			else
				node2 = mNodes[mSystemIndex][comp->getNode2()];

			comp->setNodes(Node::List{ node1, node2 });
		}
	}

	mLog.Log(Logger::Level::INFO) << "Maximum node number: " << maxNode << std::endl;
	// virtual nodes are placed after network nodes
	UInt virtualNode = maxNode;

	// Check if component requires virtual node and if so set one
	for (auto comp : mComponents[mSystemIndex]) {
		if (comp->hasVirtualNodes()) {
			for (Int node = 0; node < comp->getVirtualNodesNum(); node++) {
				virtualNode++;
				mNodes[mSystemIndex].push_back(std::make_shared<Node>(virtualNode));
				comp->setVirtualNodeAt(mNodes[mSystemIndex][virtualNode], node);
				mLog.Log(Logger::Level::INFO) << "Created virtual node" << node << "= " << virtualNode
					<< " for " << comp->getName() << std::endl;
			}
		}
	}

	// Calculate system size and create matrices and vectors
	mNumNodes = virtualNode + 1;
	createEmptyVectors();
	createEmptySystemMatrix();

	// Initialize right side vector and components
	mLog.Log(Logger::Level::INFO) << "Initialize power flow" << std::endl;
	for (auto comp : newComponents) {
		comp->initializePowerflow(mSystemFrequency);
		comp->mnaInitialize(mSystemOmega, mTimeStep);
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

void MnaSimulation::addSystemTopology(Component::List newComponents) {
	mComponents.push_back(newComponents);

	// It is assumed that the system size does not change
	createEmptySystemMatrix();
	for (auto comp : newComponents)
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

Int MnaSimulation::step(bool blocking) {
	mRightSideVector.setZero();

	for (auto eif : mExternalInterfaces) {
		eif->readValues(blocking);
	}

	for (auto comp : mComponents[mSystemIndex]) {
		comp->mnaStep(mSystemMatrices[mSystemIndex], mRightSideVector, mLeftSideVector, mTime);
	}

	solve();

	for (auto comp : mComponents[mSystemIndex]) {
		comp->mnaPostStep(mRightSideVector, mLeftSideVector, mTime);
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

	mLeftVectorLog.LogNodeValues(getTime(), getLeftSideVector());
	mRightVectorLog.LogNodeValues(getTime(), getRightSideVector());
	return mTime < mFinalTime;
}

void MnaSimulation::run() {
	mLog.Log(Logger::Level::INFO) << "Start simulation." << std::endl;

	while (step()) {
		increaseByTimeStep();
	}

	mLog.Log(Logger::Level::INFO) << "Simulation finished." << std::endl;
}

void MnaSimulation::run(double duration) {
	mLog.Log(Logger::Level::INFO) << "Run simulation for " << duration << " seconds." << std::endl;
	double started = mTime;

	while (step()) {
		increaseByTimeStep();

		if (mTime - started > duration)
			break;
	}
	mLog.Log(Logger::Level::INFO) << "Simulation finished." << std::endl;
}

void MnaSimulation::setSwitchTime(Real switchTime, Int systemIndex) {
	SwitchConfiguration newSwitchConf;
	newSwitchConf.switchTime = switchTime;
	newSwitchConf.systemIndex = systemIndex;
	mSwitchEvents.push_back(newSwitchConf);
}
