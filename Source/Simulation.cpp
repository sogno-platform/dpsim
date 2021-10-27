/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <chrono>
#include <iomanip>
#include <algorithm>
#include <typeindex>

#include <dpsim/SequentialScheduler.h>
#include <dpsim/Simulation.h>
#include <dpsim/Utils.h>
#include <cps/Utils.h>
#include <dpsim/MNASolverFactory.h>
#ifdef WITH_SPARSE
#include <dpsim/MNASolverSysRecomp.h>
#endif
#include <dpsim/PFSolverPowerPolar.h>
#include <dpsim/DiakopticsSolver.h>

#include <spdlog/sinks/stdout_color_sinks.h>

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
	mName(name),
	mLogLevel(logLevel) {
	create();
}

Simulation::Simulation(String name, CommandLineArgs& args) :
	mName(name),
	mFinalTime(args.duration),
	mTimeStep(args.timeStep),
	mLogLevel(args.logLevel),
	mDomain(args.solver.domain),
	mSolverType(args.solver.type),
	mMnaImpl(args.mnaImpl) {
	create();
}

void Simulation::create() {
	// Logging
	mLog = Logger::get(mName, mLogLevel, std::max(Logger::Level::info, mLogLevel));

	// Attributes
	addAttribute<String>("name", &mName, Flags::read);
	addAttribute<Real>("time_step", &mTimeStep, Flags::read);
	addAttribute<Real>("final_time", &mFinalTime, Flags::read|Flags::write);
	addAttribute<Bool>("steady_state_init", &mSteadyStateInit, Flags::read|Flags::write);
	addAttribute<Bool>("split_subnets", &mSplitSubnets, Flags::read|Flags::write);
	addAttribute<Real>("time_step", &mTimeStep, Flags::read);

	Eigen::setNbThreads(1);

	mInitialized = false;
}

void Simulation::initialize() {
	if (mInitialized)
		return;

	mSolvers.clear();

	switch (mDomain) {
	case Domain::SP:
		// Treat SP as DP
	case Domain::DP:
		createSolvers<Complex>();
		break;
	case Domain::EMT:
		createSolvers<Real>();
		break;
	}

	mTime = 0;
	mTimeStepCount = 0;

	schedule();

	mInitialized = true;
}

template <typename VarType>
void Simulation::createSolvers() {
	Solver::Ptr solver;
	switch (mSolverType) {
		case Solver::Type::MNA:
			createMNASolver<VarType>();
			break;
#ifdef WITH_SUNDIALS
		case Solver::Type::DAE:
			solver = std::make_shared<DAESolver>(mName, mSystem, mTimeStep, 0.0);
			mSolvers.push_back(solver);
			break;
#endif /* WITH_SUNDIALS */
		case Solver::Type::NRP:
			solver = std::make_shared<PFSolverPowerPolar>(mName, mSystem, mTimeStep, mLogLevel);
			solver->doInitFromNodesAndTerminals(mInitFromNodesAndTerminals);
			solver->initialize();
			mSolvers.push_back(solver);
			break;
		default:
			throw UnsupportedSolverException();
	}

	// Some components require a dedicated ODE solver.
	// This solver is independent of the system solver.
#ifdef WITH_SUNDIALS
	for (auto comp : mSystem.mComponents) {
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
void Simulation::createMNASolver() {
	Solver::Ptr solver;
	std::vector<SystemTopology> subnets;
	// The Diakoptics solver splits the system at a later point.
	// That is why the system is not split here if tear components exist.
	if (mSplitSubnets && mTearComponents.size() == 0)
		mSystem.splitSubnets<VarType>(subnets);
	else
		subnets.push_back(mSystem);

	for (UInt net = 0; net < subnets.size(); ++net) {
		String copySuffix;
	   	if (subnets.size() > 1)
			copySuffix = "_" + std::to_string(net);

		// TODO: In the future, here we could possibly even use different
		// solvers for different subnets if deemed useful
		if (mTearComponents.size() > 0) {
			// Tear components available, use diakoptics
			solver = std::make_shared<DiakopticsSolver<VarType>>(mName,
				subnets[net], mTearComponents, mTimeStep, mLogLevel);
		}
		else if (mSystemMatrixRecomputation) {
#ifdef WITH_SPARSE
			// Recompute system matrix if switches or other components change
			solver = std::make_shared<MnaSolverSysRecomp<VarType>>(
				mName + copySuffix, mDomain, mLogLevel);
			solver->setTimeStep(mTimeStep);
			solver->doSteadyStateInit(mSteadyStateInit);
			solver->setSteadStIniTimeLimit(mSteadStIniTimeLimit);
			solver->setSteadStIniAccLimit(mSteadStIniAccLimit);
			solver->setSystem(subnets[net]);
			solver->initialize();
#else
			throw SystemError("Recomputation Solver requires WITH_SPARSE to be set.");
#endif
		}
		else {
			// Default case with precomputed system matrices for different configurations
			solver = MnaSolverFactory::factory<VarType>(mName + copySuffix, mDomain,
												 mLogLevel, mMnaImpl);
			solver->setTimeStep(mTimeStep);
			solver->doSteadyStateInit(mSteadyStateInit);
			solver->doFrequencyParallelization(mFreqParallel);
			solver->setSteadStIniTimeLimit(mSteadStIniTimeLimit);
			solver->setSteadStIniAccLimit(mSteadStIniAccLimit);
			solver->setSystem(subnets[net]);
			solver->initialize();
		}
		mSolvers.push_back(solver);
	}
}

void Simulation::sync() {
	int numOfSyncInterfaces = std::count_if(mInterfaces.begin(), mInterfaces.end(), [](InterfaceMapping ifm) {return ifm.syncStart;});
	mLog->info("Start synchronization with remotes on {} interfaces", numOfSyncInterfaces);

	for (auto ifm : mInterfaces) {
		if(ifm.syncStart) {
			// Send initial state over interface
			ifm.interface->writeValues();

			// Blocking wait for interface
			ifm.interface->readValues(ifm.syncStart);

			ifm.interface->writeValues();
		}
	}

	mLog->info("Synchronized simulation start with remotes");
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

	for (auto intfm : mInterfaces) {
		for (auto t : intfm.interface->getTasks()) {
			mTasks.push_back(t);
		}
	}

	for (auto logger : mLoggers) {
		mTasks.push_back(logger->getTask());
	}
	if (!mScheduler) {
		mScheduler = std::make_shared<SequentialScheduler>();
	}
	mScheduler->resolveDeps(mTasks, mTaskInEdges, mTaskOutEdges);
}

void Simulation::schedule() {
	mLog->info("Scheduling tasks.");
	prepSchedule();
	mScheduler->createSchedule(mTasks, mTaskInEdges, mTaskOutEdges);
	mLog->info("Scheduling done.");
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
				mLog->info("{} {}", task->toString(), CPS::Utils::Rgb::gradient(grad).hex());
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

void Simulation::start() {
	mLog->info("Initialize simulation: {}", mName);
	if (!mInitialized)
		initialize();

	mLog->info("Opening interfaces.");

	for (auto ifm : mInterfaces)
		ifm.interface->open(mLog);

	sync();

	mLog->info("Start simulation: {}", mName);
	mLog->info("Time step: {:e}", mTimeStep);
	mLog->info("Final time: {:e}", mFinalTime);
}

void Simulation::stop() {
	mScheduler->stop();

	for (auto ifm : mInterfaces)
		ifm.interface->close();

	for (auto lg : mLoggers)
		lg->close();

	mLog->info("Simulation finished.");
	mLog->flush();
}

Real Simulation::next() {
	if (mTime < mFinalTime)
		step();
	else
		stop();

	return mTime;
}


void Simulation::run() {
	start();

	while (mTime < mFinalTime) {
		step();
	}

	stop();
}

Real Simulation::step() {
	auto start = std::chrono::steady_clock::now();
	mEvents.handleEvents(mTime);

	mScheduler->step(mTime, mTimeStepCount);

	mTime += mTimeStep;
	++mTimeStepCount;

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

void Simulation::logStepTimes(String logName) {
	auto stepTimeLog = Logger::get(logName, Logger::Level::info);
	Logger::setLogPattern(stepTimeLog, "%v");
	stepTimeLog->info("step_time");

	Real stepTimeSum = 0;
	for (auto meas : mStepTimes) {
		stepTimeSum += meas;
		stepTimeLog->info("{:f}", meas);
	}
	mLog->info("Average step time: {:.6f}", stepTimeSum / mStepTimes.size());
}

void Simulation::setIdObjAttr(const String &comp, const String &attr, Real value) {
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (compObj) {
		try {
			compObj->attribute<Real>(attr)->set(value);
		} catch (InvalidAttributeException &e) {
			mLog->error("Attribute not found");
		}
	}
	else
		mLog->error("Component not found");
}

void Simulation::setIdObjAttr(const String &comp, const String &attr, Complex value){
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (compObj) {
		try {
			compObj->attributeComplex(attr)->set(value);
		} catch (InvalidAttributeException &e) {
			mLog->error("Attribute not found");
		}
	}
	else
		mLog->error("Component not found");
}

Real Simulation::getRealIdObjAttr(const String &comp, const String &attr, UInt row, UInt col) {
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (!compObj) compObj = mSystem.node<IdentifiedObject>(comp);

	if (compObj) {
		try {
			return compObj->attribute<Real>(attr)->getByValue();
		} catch (InvalidAttributeException &e) { }

		try {
			return compObj->attributeMatrixReal(attr)->coeff(row, col)->getByValue();
		} catch (InvalidAttributeException &e) { }

		mLog->error("Attribute not found");
	}
	else {
		mLog->error("Component not found");
	}

	return 0;
}

Complex Simulation::getComplexIdObjAttr(const String &comp, const String &attr, UInt row, UInt col) {
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (!compObj) compObj = mSystem.node<IdentifiedObject>(comp);

	if (compObj) {
		try {
			return compObj->attributeComplex(attr)->getByValue();
		} catch (InvalidAttributeException &e) { }

		try {
			return compObj->attributeMatrixComp(attr)->coeff(row, col)->getByValue();
		} catch (InvalidAttributeException &e) { }

		mLog->error("Attribute not found");
	}
	else {
		mLog->error("Component not found");
	}

	return 0;
}

void Simulation::exportIdObjAttr(const String &comp, const String &attr, UInt idx, UInt row, UInt col, Complex scale, Interface* intf) {
	if (intf == nullptr) {
		intf = mInterfaces[0].interface;
	}
	
	Bool found = false;
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (!compObj) compObj = mSystem.node<TopologicalNode>(comp);

	if (compObj) {
		try {
			auto v = compObj->attribute<Real>(attr);

			if (scale != Complex(1, 0)) {
				throw TypeException();
			}

			intf->exportReal(v, idx);
			found = true;
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeComplex(attr)->scale(scale);
			intf->exportComplex(v, idx);
			found = true;
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeMatrixReal(attr)->coeff(row, col);
			if (scale != Complex(1, 0)) {
				throw TypeException();
			}
			intf->exportReal(v, idx);
			found = true;
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeMatrixComp(attr);
			intf->exportComplex(v->coeff(row, col)->scale(scale), idx);
			found = true;
		} catch (InvalidAttributeException &e) { }

		if (!found) mLog->error("Attribute not found");
	}
	else {
		mLog->error("Component not found");
	}
}

void Simulation::exportIdObjAttr(const String &comp, const String &attr, UInt idx, AttributeBase::Modifier mod, UInt row, UInt col, Interface* intf) {
	if(intf == nullptr) {
		intf = mInterfaces[0].interface;
	}
	
	Bool found = false;
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (!compObj) compObj = mSystem.node<TopologicalNode>(comp);

	auto name = fmt::format("{}.{}", comp, attr);

	if (compObj) {
		try {
			auto v = compObj->attribute<Real>(attr);
			intf->exportReal(v, idx);
			found = true;
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeComplex(attr);
			switch(mod) {
				case AttributeBase::Modifier::real :
					intf->exportReal(v->real(), idx, name);
					found = true;
					break;
				case AttributeBase::Modifier::imag :
					intf->exportReal(v->imag(), idx, name);
					found = true;
					break;
				case AttributeBase::Modifier::mag :
					intf->exportReal(v->mag(), idx, name);
					found = true;
					break;
				case AttributeBase::Modifier::phase :
					intf->exportReal(v->phase(), idx, name);
					found = true;
					break;
			}
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeMatrixReal(attr)->coeff(row, col);
			intf->exportReal(v, idx);
			found = true;
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeMatrixComp(attr);
			switch(mod) {
				case AttributeBase::Modifier::real :
					intf->exportReal(v->coeffReal(row, col), idx, name);
					found = true;
					break;
				case AttributeBase::Modifier::imag :
					intf->exportReal(v->coeffImag(row, col), idx, name);
					found = true;
					break;
				case AttributeBase::Modifier::mag :
					intf->exportReal(v->coeffMag(row, col), idx, name);
					found = true;
					break;
				case AttributeBase::Modifier::phase :
					intf->exportReal(v->coeffPhase(row, col), idx, name);
					found = true;
					break;
			}
		} catch (InvalidAttributeException &e) { }

		if (!found) mLog->error("Attribute not found");
	}
	else {
		mLog->error("Component not found");
	}
}

void Simulation::importIdObjAttr(const String &comp, const String &attr, UInt idx, Interface* intf) {
	if (intf == nullptr) {
		intf = mInterfaces[0].interface;
	}
	Bool found = false;
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	if (!compObj) compObj = mSystem.node<TopologicalNode>(comp);

	if (compObj) {
		try {
			auto v = compObj->attribute<Real>(attr);
			compObj->setAttributeRef(attr, intf->importReal(idx));
			found = true;
		} catch (InvalidAttributeException &e) { }

		try {
			auto v = compObj->attributeComplex(attr);
			compObj->setAttributeRef(attr, intf->importComplex(idx));
			found = true;
		} catch (InvalidAttributeException &e) { }

		if (!found) mLog->error("Attribute not found");
	}
	else {
		mLog->error("Component not found");
	}
}


void Simulation::logIdObjAttr(const String &comp, const String &attr) {
	IdentifiedObject::Ptr compObj = mSystem.component<IdentifiedObject>(comp);
	IdentifiedObject::Ptr nodeObj = mSystem.node<TopologicalNode>(comp);

	if (compObj) {
		try {
			auto name = compObj->name() + "." + attr;
			auto v = compObj->attribute(attr);
			mLoggers[0]->addAttribute(name, v);

		} catch (InvalidAttributeException &e) {
			mLog->error("Attribute not found");
		}
	} else if (nodeObj) {
		try {
			auto name = nodeObj->name() + "." + attr;
			auto v = nodeObj->attribute(attr);
			mLoggers[0]->addAttribute(name, v);

		} catch (InvalidAttributeException &e) {
			mLog->error("Attribute not found");
		}
	}
	else {
		mLog->error("Component not found");
	}
}
