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

#ifdef __linux__
#include <unistd.h>
#endif

#include "Simulation.h"

#ifdef WITH_CIM
  #include "cps/CIM/Reader.h"
#endif

using namespace CPS;
using namespace DPsim;

Simulation::Simulation(String name,
	Real timeStep, Real finalTime,
	Solver::Domain domain, Solver::Type solverType,
	Logger::Level logLevel) :
	mLog("Logs/" + name + ".log", logLevel),
	mName(name),
	mFinalTime(finalTime),
	mLogLevel(logLevel),
	mPipe{-1, -1}
{
	mAttributes["name"] = Attribute<String>::make(&mName, Flags::read);
	mAttributes["final_time"] = Attribute<Real>::make(&mFinalTime, Flags::read);
}

Simulation::Simulation(String name, SystemTopology system,
	Real timeStep, Real finalTime,
	Solver::Domain domain, Solver::Type solverType,
	Logger::Level logLevel,
	Bool steadyStateInit) :
	Simulation(name, timeStep, finalTime,
		domain, solverType, logLevel) {

	switch (solverType) {
	case Solver::Type::MNA:
	default:
		mSolver = std::make_shared<MnaSolver>(name,
			system, timeStep,
			domain, logLevel, steadyStateInit);
		break;
	}
}

Simulation::~Simulation() {
#ifdef __linux__
	if (mPipe[0] >= 0) {
		close(mPipe[0]);
		close(mPipe[1]);
	}
#endif
}

void Simulation::run() {
	mLog.Log(Logger::Level::INFO) << "Start simulation." << std::endl;

#ifdef WITH_SHMEM
	// We send initial state over all interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->writeValues();
	}

	// Blocking wait for interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->readValues(ifm.syncStart);
	}
#endif

	while (mTime < mFinalTime) {
		step();
	}

	mLog.Log(Logger::Level::INFO) << "Simulation finished." << std::endl;
}

Real Simulation::step() {
	Real nextTime;

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces) {
		ifm.interface->readValues(ifm.sync);
	}
#endif

	nextTime = mSolver->step(mTime);

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces) {
		ifm.interface->writeValues();
	}
#endif

	mSolver->log(mTime);
	mTime = nextTime;
	mTimeStepCount++;

	return mTime;
}

void Simulation::setSwitchTime(Real switchTime, Int systemIndex) {
	mSolver->setSwitchTime(switchTime, systemIndex);
}

void Simulation::addSystemTopology(SystemTopology system) {
	mSolver->addSystemTopology(system);
}

#ifdef __linux__
int Simulation::getEventFD(Int flags, Int coalesce) {
	int ret;

	// Create a new pipe of not existant
	if (mPipe[0] < 0) {
		ret = pipe(mPipe);
		if (ret < 0)
			throw SystemError("Failed to create pipe");
	}

	// Return read end
	return mPipe[0];
}

void Simulation::sendNotification(enum Event evt) {
	int ret;

	if (mPipe[0] < 0) {
		ret = pipe(mPipe);
		if (ret < 0)
			throw SystemError("Failed to create pipe");
	}

	uint32_t msg = static_cast<uint32_t>(evt);

	ret = write(mPipe[1], &msg, 4);
	if (ret < 0)
		throw SystemError("Failed notify");
}
#endif
