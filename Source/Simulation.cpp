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

#include <unistd.h>

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
{ }

Simulation::Simulation(String name, SystemTopology system,
	Real timeStep, Real finalTime,
	Solver::Domain domain, Solver::Type solverType,
	Logger::Level logLevel) :
	Simulation(name, timeStep, finalTime,
		domain, solverType, logLevel) {

	switch (solverType) {
	case Solver::Type::MNA:
	default:
		mSolver = std::make_shared<MnaSolver>(name,
			system, timeStep,
			domain, logLevel);
		break;
	}
}

#ifdef WITH_CIM
Simulation::Simulation(String name, std::list<String> cimFiles, Real frequency,
	Real timeStep, Real finalTime,
	Solver::Domain domain,
	Solver::Type solverType,
	Logger::Level logLevel) :
	Simulation(name, timeStep, finalTime,
		domain, solverType, logLevel) {

	CIM::Reader reader(frequency, logLevel, logLevel);
	reader.addFiles(cimFiles);
	reader.parseFiles();

	SystemTopology system = reader.getSystemTopology();

	switch (solverType) {
	case Solver::Type::MNA:
	default:
		mSolver = std::make_shared<MnaSolver>(name,
			system, timeStep,
			domain, logLevel);
		break;
	}
}
#endif

Simulation::~Simulation() {
	if (mPipe[0] >= 0) {
		close(mPipe[0]);
		close(mPipe[1]);
	}
}

void Simulation::run(bool blocking) {
	mLog.Log(Logger::Level::INFO) << "Start simulation." << std::endl;

	sendNotification(Event::STARTED);

	while (mTime < mFinalTime) {
		Real nextTime;
		nextTime = mSolver->step(mTime, blocking);
		mSolver->log(mTime);
		mTime = nextTime;
		mTimeStepCount++;
	}

	mLog.Log(Logger::Level::INFO) << "Simulation finished." << std::endl;
}

void Simulation::run(double duration, bool blocking) {
	double started = mTime;

	mLog.Log(Logger::Level::INFO) << "Run simulation for " << duration << " seconds." << std::endl;

	while ((mTime - started) < duration) {
		Real nextTime;
		nextTime = mSolver->step(mTime, blocking);
		mSolver->log(mTime);
		mTime = nextTime;
		mTimeStepCount++;
	}

	mLog.Log(Logger::Level::INFO) << "Simulation ran for " << duration << " seconds." << std::endl;
}

Real Simulation::step(bool blocking) {
	Real nextTime;

	nextTime = mSolver->step(mTime, blocking);
	mSolver->log(mTime);
	mTime = nextTime;
	mTimeStepCount++;

	return mTime;
}

void Simulation::setSwitchTime(Real switchTime, Int systemIndex) {
	mSolver->setSwitchTime(switchTime, systemIndex);
}


void Simulation::addInterface(Interface *eint) {
	mSolver->addInterface(eint);
}

void Simulation::addSystemTopology(SystemTopology system) {
	mSolver->addSystemTopology(system);
}

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
