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
#include <iomanip>
#include <algorithm>
#include <typeindex>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim/SequentialScheduler.h>
#include <dpsim/Simulation.h>
#include <dpsim/Utils.h>
#include <cps/Utils.h>
#include <dpsim/MNASolver.h>
#include <dpsim/NRPSolver.h>
#include <dpsim/DiakopticsSolver.h>

#ifdef WITH_CIM
  #include <cps/CIM/Reader.h>
#endif

#ifdef WITH_SUNDIALS
  #include <cps/Solver/ODEInterface.h>
  #include <dpsim/DAESolver.h>
  #include <dpsim/ODESolver.h>
#endif

using namespace CPS;
using namespace DPsim;

Simulation::Simulation(String name,	Logger::Level logLevel) :
	mName(name), mLogLevel(logLevel) {

	addAttribute<String>("name", &mName, Flags::read);
	addAttribute<Real>("final_time", &mFinalTime, Flags::read|Flags::write);
	addAttribute<Bool>("steady_state_init", &mSteadyStateInit, Flags::read|Flags::write);
	addAttribute<Bool>("split_subnets", &mSplitSubnets, Flags::read|Flags::write);
	addAttribute<Real>("time_step", &mTimeStep, Flags::read);

	Eigen::setNbThreads(1);

	// Logging
	mSLog = Logger::get(name, logLevel);
	mSLog->set_pattern("[%L] %v");

	mCLog = spdlog::stderr_color_mt(name + "_console");
	mCLog->set_level(logLevel);
	mCLog->set_pattern(fmt::format("{}[%T.%f %n %^%l%$] %v", CPS::Logger::prefix()));

	mInitialized = false;
}

Simulation::Simulation(String name, SystemTopology system,
	Real timeStep, Real finalTime,
	Domain domain, Solver::Type solverType,
	Logger::Level logLevel,
	Bool steadyStateInit,
	Bool splitSubnets,
	Component::List tearComponents) :
	Simulation(name, logLevel) {

	mTimeStep = timeStep;
	mFinalTime = finalTime;
	mDomain = domain;
	mSystem = system;
	mSolverType = solverType;
	mSteadyStateInit = steadyStateInit;
	mSplitSubnets = splitSubnets;
	mTearComponents = tearComponents;

	mInitialized = false;
}

void Simulation::initialize() {
	if (mInitialized)
		return;

	mSolvers.clear();

	switch (mDomain) {
	case Domain::DP:
		createSolvers<Complex>(mSystem, mSolverType, mSteadyStateInit, mSplitSubnets, mTearComponents);
		break;
	case Domain::EMT:
		createSolvers<Real>(mSystem, mSolverType, mSteadyStateInit, mSplitSubnets, mTearComponents);
		break;
	case Domain::SP:
		mSolvers.push_back(std::make_shared<NRpolarSolver>(mName, mSystem, mTimeStep, mDomain, mLogLevel));
		break;
	}

	mTime = 0;
	mTimeStepCount = 0;

	schedule();

	mInitialized = true;
}

template <typename VarType>
void Simulation::splitSubnets(const SystemTopology& system, std::vector<SystemTopology>& splitSystems) {
	std::unordered_map<typename Node<VarType>::Ptr, int> subnet;
	int numberSubnets = checkTopologySubnets<VarType>(system, subnet);
	if (numberSubnets == 1) {
		splitSystems.push_back(system);
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
			splitSystems.emplace_back(system.mSystemFrequency, nodes[currentNet], components[currentNet]);
		}
	}
}

template <typename VarType>
void Simulation::createSolvers(const CPS::SystemTopology& system, Solver::Type solverType,
	Bool steadyStateInit, Bool doSplitSubnets, const Component::List& tearComponents) {
	std::vector<SystemTopology> subnets;
	if (doSplitSubnets && tearComponents.size() == 0)
		splitSubnets<VarType>(system, subnets);
	else
		subnets.push_back(system);

	for (UInt net = 0; net < subnets.size(); net++) {
		String copySuffix;
	   	if (subnets.size() > 1)
			copySuffix = "_" + std::to_string(net);

		// TODO in the future, here we could possibly even use different
		// solvers for different subnets if deemed useful
		Solver::Ptr solver;
		switch (solverType) {
			case Solver::Type::MNA:
				if (tearComponents.size() > 0) {
					solver = std::make_shared<DiakopticsSolver<VarType>>(mName,
						subnets[net], tearComponents, mTimeStep, mLogLevel);
				} else {
					solver = std::make_shared<MnaSolver<VarType>>(mName + copySuffix,
						subnets[net], mTimeStep, mDomain, mLogLevel, steadyStateInit, mDownSampleRate, mHarmParallel);
				}
				break;
#ifdef WITH_SUNDIALS
			case Solver::Type::DAE:
				solver = std::make_shared<DAESolver>(mName + copySuffix, subnets[net], mTimeStep, 0.0);
				break;
#endif /* WITH_SUNDIALS */
			default:
				throw UnsupportedSolverException();
		}
		mSolvers.push_back(solver);
	}

#ifdef WITH_SUNDIALS
	for (auto comp : system.mComponents) {
		auto odeComp = std::dynamic_pointer_cast<ODEInterface>(comp);
		if (odeComp) {
			// TODO explicit / implicit integration
			auto odeSolver = std::make_shared<ODESolver>(
				odeComp->attribute<String>("name")->get() + "_ODE", odeComp, false, mTimeStep);
			mSolvers.push_back(odeSolver);
		}
	}
#endif /* WITH_SUNDIALS */
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

	mSLog->info("Waiting for start synchronization on {} interfaces", mInterfaces.size());

	// Blocking wait for interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->readValues(ifm.syncStart);
	}

	for (auto ifm : mInterfaces) {
		ifm.interface->writeValues();
	}

	mSLog->info("Synchronized simulation start with remotes");
#endif
}

void Simulation::prepSchedule() {
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
	if (!mScheduler) {
		mScheduler = std::make_shared<SequentialScheduler>();
	}
	mScheduler->resolveDeps(mTasks, mTaskInEdges, mTaskOutEdges);
}

void Simulation::schedule() {
	mSLog->info("Scheduling tasks.");
	prepSchedule();
	mScheduler->createSchedule(mTasks, mTaskInEdges, mTaskOutEdges);
}

#ifdef WITH_GRAPHVIZ
Graph::Graph Simulation::dependencyGraph() {
	if (!mInitialized)
		initialize();

	std::map<CPS::Task::Ptr, Scheduler::TaskTime> avgTimes;
	std::map<CPS::Task::Ptr, String> fillColors;

	/* The main SolveTasks of each Solver usually takes the
	 * largest amount of computing time. We exclude it from
	 * coloring for improving the spread of the color range
	 * for the remaining tasks.
	 */
// 	auto isSolveTask = [](Task::Ptr task) -> Bool {
// 		const std::vector<std::type_index> ignoreTasks = {
// #ifdef WITH_SUNDIALS
// 			std::type_index(typeid(ODESolver::SolveTask)),
// #endif
// 			std::type_index(typeid(MnaSolver<Real>::SolveTask)),
// 			std::type_index(typeid(MnaSolver<Complex>::SolveTask)),
// 			std::type_index(typeid(DiakopticsSolver<Real>::SubnetSolveTask)),
// 			std::type_index(typeid(DiakopticsSolver<Complex>::SubnetSolveTask)),
// 			std::type_index(typeid(DiakopticsSolver<Real>::SolveTask)),
// 			std::type_index(typeid(DiakopticsSolver<Complex>::SolveTask)),
// 			std::type_index(typeid(NRpolarSolver::SolveTask))
// 		};

// 		return std::find(ignoreTasks.begin(), ignoreTasks.end(), std::type_index(typeid(*task.get()))) != ignoreTasks.end();
// 	};

	// auto isIgnoredTask = [&isSolveTask](Task::Ptr task) -> Bool {
	// 	return typeid(*task.get()) == typeid(Interface::PreStep) || sSolveTask(task);
	// };

	// auto isRootTask = [](Task::Ptr task) -> Bool {
	// 	return typeid(*task.get()) == typeid(Scheduler::Root);
	// };

	auto isScheduled = [this](Task::Ptr task) -> Bool {
		return ! mTaskOutEdges[task].empty();
	};

	auto getColor = [](Task::Ptr task) -> String {
		static std::map<std::type_index, String> colorMap;
		auto tid = std::type_index(typeid(*task.get()));

		if (colorMap.find(tid) != colorMap.end()) {
			colorMap[tid] = String("/paired9/") + std::to_string(1 + colorMap.size() % 9);
		}

		return colorMap[tid];
	};

	auto avgTimeWorst = Scheduler::TaskTime::min();
	for (auto task : mTasks) {
		avgTimes[task] = mScheduler->getAveragedMeasurement(task);

		if (avgTimes[task] > avgTimeWorst)
			avgTimeWorst = avgTimes[task];
	}

	// TODO: For level-based Scheduler's we might want to
	//       maintain the level structure by setting the respective
	//       Graphviz 'rank' attributes and group each level into sub-graph

	Graph::Graph g("dependencies", Graph::Type::directed);
	for (auto task : mTasks) {
		String name = task->toString();
		String type = CPS::Utils::className(task.get(), "DPsim::");

		auto *n = g.addNode(name);

		std::stringstream label;

		label << "<B>" << Utils::encodeXml(name) << "</B><BR/>";
		label << "<FONT POINT-SIZE=\"10\" COLOR=\"gray28\">"
		      << Utils::encodeXml(type) << "<BR/>";

		if (isScheduled(task))
			label << "Avg. time: " << avgTimes[task].count() << "ns<BR/>";
		else
			label << "Unscheduled" << "<BR/>";

		label <<"</FONT>";

		n->set("color", getColor(task));
		n->set("label", label.str(), true);
		n->set("style", "rounded,filled,bold");
		n->set("shape", "rectangle");

		// if (isSolveTask(task)) {
		// 	n->set("fillcolor", "orange");
		// }
		// if (isRootTask(task)) {
		// 	n->set("fillcolor", "springgreen1");
		// }
		if (isScheduled(task)) {
			if (avgTimeWorst > Scheduler::TaskTime(0)) {
				auto grad = (float) avgTimes[task].count() / avgTimeWorst.count();
				n->set("fillcolor", CPS::Utils::Rgb::gradient(grad).hex());
				mSLog->info("{} {}", task->toString(), CPS::Utils::Rgb::gradient(grad).hex());
			}
		}
		else {
			n->set("fillcolor", "white");
		}

	}
	for (auto from : mTasks) {
		for (auto to : mTaskOutEdges[from]) {
			g.addEdge("", g.node(from->toString()), g.node(to->toString()));
		}
	}

	g.set("splines", "ortho");
	return g;
}
#endif

void Simulation::run() {
	if (!mInitialized)
		initialize();

#ifdef WITH_SHMEM
	mCLog->info("Opening interfaces.");

	for (auto ifm : mInterfaces)
		ifm.interface->open(mCLog);

	sync();
#endif

	mCLog->info("Start simulation.");

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

	mCLog->info("Simulation finished.");
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

void Simulation::reset() {

	// Resets component states
	mSystem.reset();

	for (auto l : mLoggers)
		l->reopen();

	// Force reinitialization for next run
	mInitialized = false;
}
