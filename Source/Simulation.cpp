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

#ifdef WITH_CIM
#include "cps/CIM/Reader.h"
#endif

using namespace CPS;
using namespace DPsim;

Simulation::Simulation(String name,
	Real timeStep, Real finalTime,
	Solver::Domain domain,
	Solver::Type solverType,
	Logger::Level logLevel,
	Bool steadyStateInit) :
	mLog("Logs/" + name + ".log", logLevel) {

	mName = name;
	mLogLevel = logLevel;
}

Simulation::Simulation(String name, SystemTopology system,
	Real timeStep, Real finalTime,
	Solver::Domain domain,
	Solver::Type solverType,
	Logger::Level logLevel) :
	Simulation(name, timeStep, finalTime,
		domain, solverType, logLevel, false) {

	switch (solverType) {
	case Solver::Type::MNA:
	default:
		mSolver = std::make_shared<MnaSolver>(name,
			system, timeStep, finalTime,
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
		domain, solverType, logLevel, true) {

	CIM::Reader reader(frequency, logLevel, logLevel);
	reader.addFiles(cimFiles);
	reader.parseFiles();

	SystemTopology system = reader.getSystemTopology();

	switch (solverType) {
	case Solver::Type::MNA:
	default:
		mSolver = std::make_shared<MnaSolver>(name,
			system, timeStep, finalTime,
			domain, logLevel);
		break;
	}
}
#endif

void Simulation::run() {
	mSolver->run();
}

void Simulation::run(double duration) {
	mSolver->run(duration);
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
