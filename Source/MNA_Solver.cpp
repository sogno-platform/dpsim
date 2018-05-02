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
#include "cps/CIM/Reader.h"
#endif /* WITH_CIM */

using namespace CPS;
using namespace DPsim;

MnaSolver::MnaSolver(String name,
	Real timeStep, Solver::Domain domain,
	Logger::Level logLevel, Bool steadyStateInit, Int downSampleRate) :
	mLog("Logs/" + name + "_MNA.log", logLevel),
	mLeftVectorLog("Logs/" + name + "_LeftVector.csv", logLevel),
	mRightVectorLog("Logs/" + name + "_RightVector.csv", logLevel) {

	mTimeStep = timeStep;
	mDomain = domain;
	mLogLevel = logLevel;
	mDownSampleRate = downSampleRate;
	mSteadyStateInit = steadyStateInit;
}

MnaSolver::MnaSolver(String name, SystemTopology system,
	Real timeStep, Solver::Domain domain,
	Logger::Level logLevel, Int downSampleRate)
	: MnaSolver(name, timeStep, domain,
		logLevel, false, downSampleRate) {
	initialize(system);

	// Logging
	for (auto comp : system.mComponents)
		mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;

	mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemMatrices[0]);
	mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mLuFactorizations[0].matrixLU());
	mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mRightSideVector);
}

void MnaSolver::initialize(SystemTopology system) {
	mSystemTopologies.push_back(system);

	mLog.Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;
	// Calculate the mNumber of nodes by going through the list of components
	// TODO we use the values from the first component vector right now and assume that
	// these values don't change on switches
	Int maxNode = 0;
	for (auto comp : mSystemTopologies[0].mComponents) {
		// determine maximum node in component list
		if (comp->getNode1() > maxNode)
			maxNode = comp->getNode1();
		if (comp->getNode2() > maxNode)
			maxNode = comp->getNode2();
	}

	if (mSystemTopologies[0].mNodes.size() == 0) {
		// Create Nodes for all node indices
		mSystemTopologies[0].mNodes.resize(maxNode + 1, nullptr);
		for (int node = 0; node < mSystemTopologies[0].mNodes.size(); node++)
			mSystemTopologies[0].mNodes[node] = std::make_shared<Node>(node);

		assignNodesToComponents(mSystemTopologies[0].mComponents);
	}

	mLog.Log(Logger::Level::INFO) << "Maximum node number: " << maxNode << std::endl;
	// virtual nodes are placed after network nodes
	UInt virtualNode = maxNode;
	mNumRealNodes = maxNode + 1;

	// Check if component requires virtual node and if so set one
	for (auto comp : mSystemTopologies[0].mComponents) {
		if (comp->hasVirtualNodes()) {
			for (Int node = 0; node < comp->getVirtualNodesNum(); node++) {
				virtualNode++;
				mSystemTopologies[0].mNodes.push_back(std::make_shared<Node>(virtualNode));
				comp->setVirtualNodeAt(mSystemTopologies[0].mNodes[virtualNode], node);
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

	// TODO: Move to base solver class
	mLog.Log(Logger::Level::INFO) << "Initialize power flow" << std::endl;
	for (auto comp : mSystemTopologies[0].mComponents)
		comp->initializePowerflow(mSystemTopologies[0].mSystemFrequency);

	if (mSteadyStateInit && mDomain == Solver::Domain::DP) {
		steadyStateInitialization();
	}

	// Initialize right side vector and components
	for (auto comp : mSystemTopologies[0].mComponents) {
		comp->mnaInitialize(mSystemTopologies[0].mSystemOmega, mTimeStep);
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		comp->mnaApplySystemMatrixStamp(mSystemMatrices[0]);
	}
	// Compute LU-factorization for system matrix
	mLuFactorizations.push_back(Eigen::PartialPivLU<Matrix>(mSystemMatrices[0]));

}

// TODO: check if this works with AC sources
void MnaSolver::steadyStateInitialization() {
	Real time = 0;

	// Initialize right side vector and components
	for (auto comp : mSystemTopologies[0].mComponents) {
		comp->mnaInitialize(mSystemTopologies[0].mSystemOmega, mTimeStep);
		comp->mnaApplyRightSideVectorStamp(mRightSideVector);
		comp->mnaApplySystemMatrixStamp(mSystemMatrices[0]);
		comp->mnaApplyInitialSystemMatrixStamp(mSystemMatrices[0]);
	}
	// Compute LU-factorization for system matrix
	mLuFactorizations.push_back(Eigen::PartialPivLU<Matrix>(mSystemMatrices[0]));

	mLog.Log(Logger::Level::INFO) << "Start initialization." << std::endl;
	Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
	Matrix diff;
	Real maxDiff, max;

	while (time < 10) {
		time += step(time);

		diff = prevLeftSideVector - mLeftSideVector;
		prevLeftSideVector = mLeftSideVector;
		maxDiff = diff.lpNorm<Eigen::Infinity>();
		max = mLeftSideVector.lpNorm<Eigen::Infinity>();

		if ((maxDiff / max) < 0.0001)
			break;
	}
	mLog.Log(Logger::Level::INFO) << "Initialization finished. Max difference: "
		<< maxDiff << " or " << maxDiff / max << "% at time " << time << std::endl;
	mSystemMatrices.pop_back();
	mLuFactorizations.pop_back();
	mSystemIndex = 0;

	createEmptySystemMatrix();
}

void MnaSolver::assignNodesToComponents(ComponentBase::List components) {
	for (auto comp : components) {
		// Do not reassign nodes or change them
		if (comp->getNodeNum() > 0) {
			std::shared_ptr<Node> node1, node2;
			if (comp->getNode1() < 0)
				node1 = GND;
			else
				node1 = mSystemTopologies[0].mNodes[comp->getNode1()];
			if (comp->getNode2() < 0)
				node2 = GND;
			else
				node2 = mSystemTopologies[0].mNodes[comp->getNode2()];

			comp->setNodes(Node::List{ node1, node2 });
		}
	}
}

void MnaSolver::createEmptyVectors() {
	if (mDomain == Solver::Domain::EMT) {
		mRightSideVector = Matrix::Zero(mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(mNumNodes, 1);
	}
	else {
		mRightSideVector = Matrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
	}
}

void MnaSolver::createEmptySystemMatrix() {
	if (mDomain == Solver::Domain::EMT) {
		mSystemMatrices.push_back(Matrix::Zero(mNumNodes, mNumNodes));
	}
	else {
		mSystemMatrices.push_back(Matrix::Zero(2 * mNumNodes, 2 * mNumNodes));
	}
}

void MnaSolver::addSystemTopology(SystemTopology system) {
	mSystemTopologies.push_back(system);
	assignNodesToComponents(system.mComponents);

	// It is assumed that the system size does not change
	createEmptySystemMatrix();
	for (auto comp : system.mComponents)
		comp->mnaApplySystemMatrixStamp(mSystemMatrices[mSystemMatrices.size()]);

	mLuFactorizations.push_back(LUFactorized(mSystemMatrices[mSystemMatrices.size()]));
}

void MnaSolver::switchSystemMatrix(Int systemIndex) {
	if (systemIndex < mSystemMatrices.size())
		mSystemIndex = systemIndex;
}

void MnaSolver::solve()  {
	mLeftSideVector = mLuFactorizations[mSystemIndex].solve(mRightSideVector);
}

Real MnaSolver::step(Real time, bool blocking) {
	mRightSideVector.setZero();

	for (auto eif : mInterfaces) {
		eif->readValues(blocking);
	}

	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		comp->mnaStep(mSystemMatrices[mSystemIndex], mRightSideVector, mLeftSideVector, time);
	}

	solve();

	for (auto comp : mSystemTopologies[mSystemIndex].mComponents) {
		comp->mnaPostStep(mRightSideVector, mLeftSideVector, time);
	}

	for (UInt nodeIdx = 0; nodeIdx < mNumRealNodes; nodeIdx++) {
		mSystemTopologies[0].mNodes[nodeIdx]->mnaUpdateVoltages(mLeftSideVector);
	}

	for (auto eif : mInterfaces) {
		eif->writeValues();
	}

	if (mSwitchTimeIndex < mSwitchEvents.size()) {
		if (time >= mSwitchEvents[mSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEvents[mSwitchTimeIndex].systemIndex);
			++mSwitchTimeIndex;

			mLog.Log(Logger::Level::INFO) << "Switched to system " << mSwitchTimeIndex << " at " << time << std::endl;
			mLog.Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemMatrices[mSystemIndex] << std::endl;
			mLog.Log(Logger::Level::INFO) << "New decomp:" << std::endl << mLuFactorizations[mSystemIndex].matrixLU() << std::endl;
		}
	}

	return time + mTimeStep;
}

void MnaSolver::log(Real time) {
	mLeftVectorLog.LogNodeValues(time, getLeftSideVector());
	mRightVectorLog.LogNodeValues(time, getRightSideVector());
}

void MnaSolver::setSwitchTime(Real switchTime, Int systemIndex) {
	SwitchConfiguration newSwitchConf;
	newSwitchConf.switchTime = switchTime;
	newSwitchConf.systemIndex = systemIndex;
	mSwitchEvents.push_back(newSwitchConf);
}
