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

#include "Simulation.h"

#ifdef WITH_RT
  #include <signal.h>
  #include <sys/timerfd.h>
  #include <time.h>
  #include <unistd.h>
#endif /* WITH_RT */

using namespace DPsim;

Simulation::Simulation(String name, Components::Base::List elements, Real om, Real dt, Real tf, Logger::Level logLevel,
	SimulationType simType, Int downSampleRate)
{
	mTime = 0;
	mLastLogTimeStep = 0;
	mCurrentSwitchTimeIndex = 0;

	mName = name;
	mLogLevel = logLevel;
	mSystemModel.setSimType(simType);
	mSystemModel.setTimeStep(dt);
	mSystemModel.setOmega(om);
	mFinalTime = tf;
	mDownSampleRate = downSampleRate;
	mLog = std::make_shared<Logger>("Logs/" + name + ".log", mLogLevel);
	mLeftVectorLog = std::make_shared<Logger>("Logs/" + name + "_LeftVector.csv", mLogLevel);
	mRightVectorLog = std::make_shared<Logger>("Logs/" + name + "_RightVector.csv", mLogLevel);

	initialize(elements);

	for (auto c : elements) {
		mLog->Log(Logger::Level::INFO) << "Added " << c->getType() << " '" << c->getName() << "' to simulation." << std::endl;
	}

	mLog->Log(Logger::Level::INFO) << "System matrix A:" << std::endl;
	mLog->LogMatrix(Logger::Level::INFO, mSystemModel.getCurrentSystemMatrix());
	mLog->Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog->LogMatrix(Logger::Level::INFO, mSystemModel.getLUdecomp());
	mLog->Log(Logger::Level::INFO) << "Known variables matrix j:" << std::endl;
	mLog->LogMatrix(Logger::Level::INFO, mSystemModel.getRightSideVector());
}

Simulation::~Simulation() {
}

void Simulation::initialize(Components::Base::List newElements)
{
	Int maxNode = 0;
	Int currentVirtualNode = 0;

	mLog->Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;

	// Calculate the mNumber of nodes by going through the list of elements
	// TODO we use the values from the first element vector right now and assume that
	// these values don't change on switches
	for (auto element : newElements) {
		// determine maximum node in component list
		if (element->getNode1() > maxNode) {
			maxNode = element->getNode1();
		}
		if (element->getNode2() > maxNode) {
			maxNode = element->getNode2();
		}
	}

	mLog->Log(Logger::Level::INFO) << "Maximum node number: " << maxNode << std::endl;
	currentVirtualNode = maxNode;

	// Check if element requires virtual node and if so set one
	for (auto element : newElements) {
		if (element->hasVirtualNodes()) {
			for (Int node = 0; node < element->getVirtualNodesNum(); node++) {
				currentVirtualNode++;
				element->setVirtualNode(node, currentVirtualNode);
				mLog->Log(Logger::Level::INFO) << "Created virtual node"<< node << "=" << currentVirtualNode
					<< " for " << element->getName() << std::endl;
			}
		}
	}

	// Calculate size of system matrix
	//Int numNodes = maxNode + currentVirtualNode + 1;
	Int numNodes = currentVirtualNode + 1;

	// Create right and left vector
	mSystemModel.initialize(numNodes);

	// Initialize right side vector and components
	for (auto element : newElements) {
		element->initialize(mSystemModel);
		element->applyRightSideVectorStamp(mSystemModel);
	}

	// Create new system matrix and apply matrix stamps
	addSystemTopology(newElements);

	switchSystemMatrix(0);
	mElements = mElementsVector[0];
}

void Simulation::addSystemTopology(Components::Base::List newElements)
{
	mElementsVector.push_back(newElements);

	// It is assumed that the system size does not change
	mSystemModel.createEmptySystemMatrix();

	for (auto element : newElements) {
		element->applySystemMatrixStamp(mSystemModel);
	}

	mSystemModel.addSystemMatrix();
}

Int Simulation::step(bool blocking)
{
	mSystemModel.setRightSideVectorToZero();

	for (auto eif : mExternalInterfaces) {
		eif->readValues(blocking);
	}

	for (auto elm : mElements) {
		elm->step(mSystemModel, mTime);
	}

	mSystemModel.solve();

	for (auto elm : mElements) {
		elm->postStep(mSystemModel);
	}

	for (auto eif : mExternalInterfaces) {
		eif->writeValues(mSystemModel);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);
			mElements = mElementsVector[++mCurrentSwitchTimeIndex];
			mLog->Log(Logger::Level::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
			mLog->Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemModel.getCurrentSystemMatrix() << std::endl;
			mLog->Log(Logger::Level::INFO) << "New decomp:" << std::endl << mSystemModel.getLUdecomp() << std::endl;
		}
	}

	mLeftVectorLog->LogNodeValues(getTime(), getLeftSideVector());
	mRightVectorLog->LogNodeValues(getTime(), getRightSideVector());

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
}

void Simulation::run()
{
	while (step()) {
		increaseByTimeStep();
	}
}

void Simulation::switchSystemMatrix(Int systemMatrixIndex)
{
	mSystemModel.switchSystemMatrix(systemMatrixIndex);
}

void Simulation::setSwitchTime(Real switchTime, Int systemIndex)
{
	switchConfiguration newSwitchConf;
	newSwitchConf.switchTime = switchTime;
	newSwitchConf.systemIndex = systemIndex;
	mSwitchEventVector.push_back(newSwitchConf);
}

void Simulation::increaseByTimeStep()
{
	mTime = mTime + mSystemModel.getTimeStep();
}

void Simulation::addExternalInterface(ExternalInterface *eint)
{
	mExternalInterfaces.push_back(eint);
}

void Simulation::setNumericalMethod(NumericalMethod numMethod)
{
	mSystemModel.setNumMethod(numMethod);
}
