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
	Domain domain,
	Logger::Level logLevel) :
	mLog(name, logLevel),
	mName(name),
	mFinalTime(finalTime),
	mDomain(domain),
	mTimeStep(timeStep),
	mLogLevel(logLevel)
{
	addAttribute<String>("name", &mName, Flags::read);
	addAttribute<Real>("final_time", &mFinalTime, Flags::read);
	Eigen::setNbThreads(1);
}

Simulation::Simulation(String name, SystemTopology system,
	Real timeStep, Real finalTime,
	Domain domain, Solver::Type solverType,
	Logger::Level logLevel,
	Bool steadyStateInit,
	Bool splitSubnets) :
	Simulation(name, timeStep, finalTime,
		domain, logLevel) {

	if (domain == Domain::DP)
		createSolvers<Complex>(system, solverType, steadyStateInit, splitSubnets);
	else
		createSolvers<Real>(system, solverType, steadyStateInit, splitSubnets);
}

Simulation::~Simulation() {

}

template <typename VarType>
void Simulation::createSolvers(const SystemTopology& system, Solver::Type solverType, Bool steadyStateInit, Bool splitSubnets) {
	std::unordered_map<typename Node<VarType>::Ptr, int> subnet;
	int numberSubnets = checkTopologySubnets<VarType>(system, subnet);
	if (numberSubnets == 1 || !splitSubnets) {
		switch (solverType) {
			case Solver::Type::MNA:
				mSolvers.push_back(std::make_shared<MnaSolver<VarType>>(mName, system, mTimeStep,
					mDomain, mLogLevel, steadyStateInit));
				break;
#ifdef WITH_SUNDIALS
			case Solver::Type::DAE:
				mSolvers.push_back(std::make_shared<DAESolver>(mName, system, mTimeStep, 0.0));
				break;
#endif /* WITH_SUNDIALS */
			default:
				throw UnsupportedSolverException();
		}
	} else {
		std::vector<Component::List> components(numberSubnets);
		std::vector<TopologicalNode::List> nodes(numberSubnets);

		// Split nodes into subnet groups
		for (auto node : system.mNodes) {
			auto pnode = std::dynamic_pointer_cast<Node<VarType>>(node);
			if (!pnode || node->isGround())
				continue;

			nodes[subnet[pnode]].push_back(node);
		}

		// Split components into subnet groups
		for (auto comp : system.mComponents) {
			auto pcomp = std::dynamic_pointer_cast<PowerComponent<VarType>>(comp);
			if (!pcomp) {
				// TODO this should only be signal components.
				// Proper solution would be to pass them to a different "solver"
				// since they are actually independent of which solver we use
				// for the electric part.
				// Just adding them to an arbitrary solver for now has the same effect.
				components[0].push_back(comp);
				continue;
			}
			for (UInt nodeIdx = 0; nodeIdx < pcomp->terminalNumber(); nodeIdx++) {
				if (!pcomp->node(nodeIdx)->isGround()) {
					components[subnet[pcomp->node(nodeIdx)]].push_back(comp);
					break;
				}
			}
		}

		for (int currentNet = 0; currentNet < numberSubnets; currentNet++) {
			SystemTopology partSys(system.mSystemFrequency, nodes[currentNet], components[currentNet]);
			String copySuffix = "_" + std::to_string(currentNet);
			// TODO in the future, here we could possibly even use different
			// solvers for different subnets if deemed useful
			switch (solverType) {
				case Solver::Type::MNA:
					mSolvers.push_back(std::make_shared<MnaSolver<VarType>>(mName + copySuffix, partSys, mTimeStep,
						mDomain, mLogLevel, steadyStateInit));
					break;
#ifdef WITH_SUNDIALS
				case Solver::Type::DAE:
					mSolvers.push_back(std::make_shared<DAESolver>(mName + copySuffix, partSys, mTimeStep, 0.0));
					break;
#endif /* WITH_SUNDIALS */
				default:
					throw UnsupportedSolverException();
			}
		}
	}

}

template <typename VarType>
int Simulation::checkTopologySubnets(const SystemTopology& system, std::unordered_map<typename Node<VarType>::Ptr, int>& subnet) {
	std::unordered_map<typename Node<VarType>::Ptr, typename Node<VarType>::List> neighbours;

	for (auto comp : system.mComponents) {
		auto pcomp = std::dynamic_pointer_cast<PowerComponent<VarType>>(comp);
		if (!pcomp)
			continue;
		for (UInt nodeIdx1 = 0; nodeIdx1 < pcomp->terminalNumberConnected(); nodeIdx1++) {
			for (UInt nodeIdx2 = 0; nodeIdx2 < nodeIdx1; nodeIdx2++) {
				auto node1 = pcomp->node(nodeIdx1);
				auto node2 = pcomp->node(nodeIdx2);
				if (node1->isGround() || node2->isGround())
					continue;
				neighbours[node1].push_back(node2);
				neighbours[node2].push_back(node1);
			}
		}
	}

	int currentNet = 0;
	size_t totalNodes = system.mNodes.size();
	for (auto tnode : system.mNodes) {
		auto node = std::dynamic_pointer_cast<Node<VarType>>(tnode);
		if (!node || tnode->isGround()) {
			totalNodes--;
		}
	}

	while (subnet.size() != totalNodes) {
		std::list<typename Node<VarType>::Ptr> nextSet;

		for (auto tnode : system.mNodes) {
			auto node = std::dynamic_pointer_cast<Node<VarType>>(tnode);
			if (!node || tnode->isGround())
				continue;

			if (subnet.find(node) == subnet.end()) {
				nextSet.push_back(node);
				break;
			}
		}
		while (!nextSet.empty()) {
			auto node = nextSet.front();
			nextSet.pop_front();

			subnet[node] = currentNet;
			for (auto neighbour : neighbours[node]) {
				if (subnet.find(neighbour) == subnet.end())
					nextSet.push_back(neighbour);
			}
		}
		currentNet++;
	}
	return currentNet;
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
	for (auto solver : mSolvers) {
		for (auto t : solver->getTasks()) {
			mTasks.push_back(t);
		}
	}
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

	mScheduler->stop();

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces)
		ifm.interface->close();
#endif

	for (auto lg : mLoggers)
		lg->close();

	mLog.info() << "Simulation finished." << std::endl;
	//mScheduler->getMeasurements();
}

Real Simulation::step() {
	auto start = std::chrono::steady_clock::now();
	mEvents.handleEvents(mTime);

	mScheduler->step(mTime, mTimeStepCount);

	mTime += mTimeStep;
	mTimeStepCount++;

	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	mStepTimes.push_back(diff.count());
	return mTime;
}
