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

#include <chrono>

#include <dpsim/SequentialScheduler.h>
#include <dpsim/Simulation.h>
#include <dpsim/MNASolver.h>

#ifdef WITH_CIM
  #include <cps/CIM/Reader.h>
#endif

#ifdef WITH_GRAPHVIZ
  #include <cps/Graph.h>
#endif

#ifdef WITH_SUNDIALS
  #include <dpsim/DAESolver.h>
#endif

using namespace CPS;
using namespace DPsim;

Simulation::Simulation(String name,
	Real timeStep, Real finalTime,
	Domain domain, Solver::Type solverType,
	Logger::Level logLevel) :
	mLog(name, logLevel),
	mName(name),
	mFinalTime(finalTime),
	mTimeStep(timeStep),
	mLogLevel(logLevel)
{
	addAttribute<String>("name", &mName, Flags::read);
	addAttribute<Real>("final_time", &mFinalTime, Flags::read);
	addAttribute<Real>("last_step_time", &mLastStepTime, Flags::read);
}

Simulation::Simulation(String name, SystemTopology system,
	Real timeStep, Real finalTime,
	Domain domain, Solver::Type solverType,
	Logger::Level logLevel,
	Bool steadyStateInit) :
	Simulation(name, timeStep, finalTime,
		domain, solverType, logLevel) {

	// TODO: multiple solver support
	switch (solverType) {
	case Solver::Type::MNA:
		if (domain == Domain::DP)
			mSolver = std::make_shared<MnaSolver<Complex>>(name, system, timeStep,
				domain, logLevel, steadyStateInit);
		else
			mSolver = std::make_shared<MnaSolver<Real>>(name, system, timeStep,
				domain, logLevel, steadyStateInit);
		break;

#ifdef WITH_SUNDIALS
	case Solver::Type::DAE:
		mSolver = std::make_shared<DAESolver>(name, system, timeStep, 0.0);
		break;
#endif /* WITH_SUNDIALS */

	default:
		throw UnsupportedSolverException();
	}

}

Simulation::~Simulation() {

}

void Simulation::sync() {
#ifdef WITH_SHMEM
	// We send initial state over all interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->writeValues();
	}

	std::cout << Logger::prefix() << "Waiting for start synchronization on " << mInterfaces.size() << " interfaces" << std::endl;

	// Blocking wait for interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->readValues(ifm.syncStart);
	}

	std::cout << Logger::prefix() << "Synchronized simulation start with remotes" << std::endl;
#endif
}

void Simulation::schedule() {
	mLog.info() << "Scheduling tasks." << std::endl;
	if (!mScheduler) {
		mScheduler = std::make_shared<SequentialScheduler>();
	}
	mTasks.clear();
	mTaskOutEdges.clear();
	mTaskInEdges.clear();
	mTasks = mSolver->getTasks();
#ifdef WITH_SHMEM
	for (auto intfm : mInterfaces) {
		for (auto t : intfm.interface->getTasks()) {
			mTasks.push_back(t);
		}
	}
#endif
	for (auto logger : mLoggers) {
		mTasks.push_back(logger->getTask());
	}

	Scheduler::resolveDeps(mTasks, mTaskInEdges, mTaskOutEdges);
	mScheduler->createSchedule(mTasks, mTaskInEdges, mTaskOutEdges);
}

void Simulation::renderDependencyGraph(std::ostream &os) {
	if (mTasks.size() == 0)
		schedule();

	Graph::Graph g("dependencies", Graph::Type::directed);
	for (auto task : mTasks) {
		g.addNode(task->toString());
	}
	for (auto from : mTasks) {
		for (auto to : mTaskOutEdges[from]) {
			g.addEdge("", g.node(from->toString()), g.node(to->toString()));
		}
	}

	g.render(os, "dot", "svg");
}

void Simulation::run() {
	schedule();

	mLog.info() << "Opening interfaces." << std::endl;

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces)
		ifm.interface->open();
#endif

	sync();

	mLog.info() << "Start simulation." << std::endl;

	while (mTime < mFinalTime) {
		step();
	}

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces)
		ifm.interface->close();
#endif

	for (auto lg : mLoggers)
		lg->flush();

	mLog.info() << "Simulation finished." << std::endl;
	mScheduler->getMeasurements();
}

Real Simulation::step() {
	auto start = std::chrono::steady_clock::now();
	mEvents.handleEvents(mTime);

	mScheduler->step(mTime, mTimeStepCount);

	mTime += mTimeStep;
	mTimeStepCount++;

	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	mLastStepTime = diff.count();
	return mTime;
}
