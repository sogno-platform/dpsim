// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <typeindex>

#include <dpsim-models/Utils.h>
#include <dpsim/DiakopticsSolver.h>
#include <dpsim/IterativeMnaSolverDirect.h>
#include <dpsim/MNASolverFactory.h>
#include <dpsim/PFSolverPowerPolar.h>
#include <dpsim/SequentialScheduler.h>
#include <dpsim/Simulation.h>
#include <dpsim/Utils.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#ifdef WITH_CIM
#include <dpsim-models/CIM/Reader.h>
#endif

#ifdef WITH_SUNDIALS
#include <dpsim-models/Solver/ODEInterface.h>
#include <dpsim/DAESolver.h>
#include <dpsim/ODESolver.h>
#endif

using namespace CPS;
using namespace DPsim;

Simulation::Simulation(String name, Logger::Level logLevel)
    : mName(AttributeStatic<String>::make(name)),
      mFinalTime(AttributeStatic<Real>::make(0.001)),
      mTimeStep(AttributeStatic<Real>::make(0.001)),
      mSplitSubnets(AttributeStatic<Bool>::make(true)),
      mSteadyStateInit(AttributeStatic<Bool>::make(false)),
      mLogLevel(logLevel) {
  create();
}

Simulation::Simulation(String name, CommandLineArgs &args)
    : mName(AttributeStatic<String>::make(name)),
      mSolverPluginName(args.solverPluginName),
      mFinalTime(AttributeStatic<Real>::make(args.duration)),
      mTimeStep(AttributeStatic<Real>::make(args.timeStep)),
      mSplitSubnets(AttributeStatic<Bool>::make(true)),
      mSteadyStateInit(AttributeStatic<Bool>::make(false)),
      mLogLevel(args.logLevel), mDomain(args.solver.domain),
      mSolverType(args.solver.type), mDirectImpl(args.directImpl) {
  create();
}

void Simulation::create() {
  // Logging
  mLog =
      Logger::get(**mName, mLogLevel, std::max(Logger::Level::info, mLogLevel));

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

template <typename VarType> void Simulation::createSolvers() {
  Solver::Ptr solver;
  switch (mSolverType) {
  case Solver::Type::MNA:
    createMNASolver<VarType>();
    break;
  case Solver::Type::ITERATIVEMNA:
    createIterativeMNASolver<VarType>();
    break;
#ifdef WITH_SUNDIALS
  case Solver::Type::DAE:
    solver = std::make_shared<DAESolver>(**mName, mSystem, **mTimeStep, 0.0);
    mSolvers.push_back(solver);
    break;
#endif /* WITH_SUNDIALS */
  case Solver::Type::NRP:
    solver = std::make_shared<PFSolverPowerPolar>(**mName, mSystem, **mTimeStep,
                                                  mLogLevel);
    solver->doInitFromNodesAndTerminals(mInitFromNodesAndTerminals);
    solver->setSolverAndComponentBehaviour(mSolverBehaviour);
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
          odeComp->mAttributeList->attributeTyped<String>("name")->get() +
              "_ODE",
          odeComp, false, **mTimeStep);
      mSolvers.push_back(odeSolver);
    }
  }
#endif /* WITH_SUNDIALS */
}

template <typename VarType> void Simulation::createMNASolver() {
  Solver::Ptr solver;
  std::vector<SystemTopology> subnets;
  // The Diakoptics solver splits the system at a later point.
  // That is why the system is not split here if tear components exist.
  if (**mSplitSubnets && mTearComponents.size() == 0)
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
      solver = std::make_shared<DiakopticsSolver<VarType>>(
          **mName, subnets[net], mTearComponents, **mTimeStep, mLogLevel);
    } else {
      // Default case with lu decomposition from mna factory
      solver = MnaSolverFactory::factory<VarType>(**mName + copySuffix, mDomain,
                                                  mLogLevel, mDirectImpl,
                                                  mSolverPluginName);
      solver->setTimeStep(**mTimeStep);
      solver->doSteadyStateInit(**mSteadyStateInit);
      solver->doFrequencyParallelization(mFreqParallel);
      solver->setSteadStIniTimeLimit(mSteadStIniTimeLimit);
      solver->setSteadStIniAccLimit(mSteadStIniAccLimit);
      solver->setSystem(subnets[net]);
      solver->setSolverAndComponentBehaviour(mSolverBehaviour);
      solver->doInitFromNodesAndTerminals(mInitFromNodesAndTerminals);
      solver->doSystemMatrixRecomputation(mSystemMatrixRecomputation);
      solver->setDirectLinearSolverConfiguration(
          mDirectLinearSolverConfiguration);
      solver->initialize();
      solver->setMaxNumberOfIterations(mMaxIterations);
    }
    mSolvers.push_back(solver);
  }
}

template <typename VarType> void Simulation::createIterativeMNASolver() {
  Solver::Ptr solver;

  std::shared_ptr<IterativeMnaSolverDirect<VarType>> Itsolver =
      std::make_shared<IterativeMnaSolverDirect<VarType>>(**mName, mDomain,
                                                          mLogLevel);
  Itsolver->setDirectLinearSolverImplementation(
      DirectLinearSolverImpl::SparseLU);

  Itsolver->setTimeStep(**mTimeStep);
  Itsolver->doSteadyStateInit(**mSteadyStateInit);
  Itsolver->doFrequencyParallelization(false);
  Itsolver->setSteadStIniTimeLimit(mSteadStIniTimeLimit);
  Itsolver->setSteadStIniAccLimit(mSteadStIniAccLimit);
  Itsolver->setSystem(mSystem);
  Itsolver->setSolverAndComponentBehaviour(mSolverBehaviour);
  Itsolver->doInitFromNodesAndTerminals(mInitFromNodesAndTerminals);
  Itsolver->doSystemMatrixRecomputation(true);
  Itsolver->setDirectLinearSolverConfiguration(
      mDirectLinearSolverConfiguration);
  Itsolver->initialize();

  mSolvers.push_back(Itsolver);
}

void Simulation::sync() const {
  SPDLOG_LOGGER_INFO(mLog, "Start synchronization with remotes on interfaces");

  for (auto intf : mInterfaces) {
    intf->syncExports();
    intf->syncImports();
    intf->syncExports();
  }

  SPDLOG_LOGGER_INFO(mLog, "Synchronized simulation start with remotes");
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

  for (auto intf : mInterfaces) {
    for (auto t : intf->getTasks()) {
      mTasks.push_back(t);
    }
  }

  for (auto logger : mLoggers) {
    mTasks.push_back(logger->getTask());
  }
  if (!mScheduler) {
    mScheduler = std::make_shared<SequentialScheduler>("taskTimes");
  }
  mScheduler->resolveDeps(mTasks, mTaskInEdges, mTaskOutEdges);
}

void Simulation::schedule() {
  SPDLOG_LOGGER_INFO(mLog, "Scheduling tasks.");
  prepSchedule();
  mScheduler->createSchedule(mTasks, mTaskInEdges, mTaskOutEdges);
  SPDLOG_LOGGER_INFO(mLog, "Scheduling done.");
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
    return !mTaskOutEdges[task].empty();
  };

  auto getColor = [](Task::Ptr task) -> String {
    static std::map<std::type_index, String> colorMap;
    auto tid = std::type_index(typeid(task.get()));

    if (colorMap.find(tid) != colorMap.end()) {
      colorMap[tid] =
          String("/paired9/") + std::to_string(1 + colorMap.size() % 9);
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
      label << "Unscheduled"
            << "<BR/>";

    label << "</FONT>";

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
        auto grad = (float)avgTimes[task].count() / avgTimeWorst.count();
        n->set("fillcolor", CPS::Utils::Rgb::gradient(grad).hex());
        SPDLOG_LOGGER_INFO(mLog, "{} {}", task->toString(),
                           CPS::Utils::Rgb::gradient(grad).hex());
      }
    } else {
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
  SPDLOG_LOGGER_INFO(mLog, "Initialize simulation: {}", **mName);
  if (!mInitialized)
    initialize();

  SPDLOG_LOGGER_INFO(mLog, "Opening interfaces.");

  for (auto intf : mInterfaces)
    intf->open();

  sync();

  SPDLOG_LOGGER_INFO(mLog, "Start simulation: {}", **mName);
  SPDLOG_LOGGER_INFO(mLog, "Time step: {:e}", **mTimeStep);
  SPDLOG_LOGGER_INFO(mLog, "Final time: {:e}", **mFinalTime);

  // In PF we dont log the initial conditions of the componentes because they are not calculated
  // In dynamic simulations log initial values of attributes (t=0)
  if (mSolverType != Solver::Type::NRP) {
    if (mLoggers.size() > 0)
      mLoggers[0]->log(0, 0);

    // In dynamic simulations increase simulation time to calculate first results at t=timestep
    mTime += **mTimeStep;
  }

  mSimulationStartTimePoint = std::chrono::steady_clock::now();
}

void Simulation::stop() {

  mSimulationEndTimePoint = std::chrono::steady_clock::now();
  mSimulationCalculationTime =
      mSimulationEndTimePoint - mSimulationStartTimePoint;
  SPDLOG_LOGGER_INFO(mLog, "Simulation calculation time: {:.6f}",
                     mSimulationCalculationTime.count());

  mScheduler->stop();

  for (auto intf : mInterfaces)
    intf->close();

  for (auto lg : mLoggers)
    lg->close();

  SPDLOG_LOGGER_INFO(mLog, "Simulation finished.");
  mLog->flush();
}

Real Simulation::next() {
  if (mTime < **mFinalTime + DOUBLE_EPSILON)
    step();
  else
    stop();

  return mTime;
}

void Simulation::run() {
  start();

  while (mTime < **mFinalTime + DOUBLE_EPSILON) {
    step();
  }

  stop();
}

Real Simulation::step() {
  auto start = std::chrono::steady_clock::now();

  mEvents.handleEvents(mTime);
  mScheduler->step(mTime, mTimeStepCount);

  mTime += **mTimeStep;
  ++mTimeStepCount;

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - start;
  mStepTimes.push_back(diff.count());
  return mTime;
}

void Simulation::logStepTimes(String logName) {
  auto stepTimeLog = Logger::get(logName, Logger::Level::info);
  Logger::setLogPattern(stepTimeLog, "%v");
  stepTimeLog->info("step_time");

  Real stepTimeSum = 0;
  for (auto meas : mStepTimes) {
    stepTimeSum += meas;
    stepTimeLog->info("{:.9f}", meas);
  }
  SPDLOG_LOGGER_INFO(mLog, "Average step time: {:.9f}",
                     stepTimeSum / mStepTimes.size());
}

void Simulation::checkForOverruns(String logName) {
  auto stepTimeLog = Logger::get(logName, Logger::Level::info);
  Logger::setLogPattern(stepTimeLog, "%v");
  stepTimeLog->info("overruns");

  int overruns = 0;
  for (auto meas : mStepTimes) {
    if (meas > **mTimeStep) {
      overruns++;
      SPDLOG_LOGGER_INFO(mLog, "overrun detected {}: {:.9f}", overruns, meas);
    }
  }
  SPDLOG_LOGGER_INFO(mLog, "Detected {} overruns.", overruns);
}

void Simulation::logLUTimes() {
  for (auto solver : mSolvers) {
    solver->logLUTimes();
  }
}

CPS::AttributeBase::Ptr Simulation::getIdObjAttribute(const String &comp,
                                                      const String &attr) {
  IdentifiedObject::Ptr idObj = mSystem.component<IdentifiedObject>(comp);
  if (!idObj) {
    idObj = mSystem.node<TopologicalNode>(comp);
  }

  if (idObj) {
    try {
      CPS::AttributeBase::Ptr attrPtr = idObj->attribute(attr);
      return attrPtr;
    } catch (InvalidAttributeException &e) {
      SPDLOG_LOGGER_ERROR(
          mLog, "Attribute with name {} not found on component {}", attr, comp);
      throw InvalidAttributeException();
    }
  } else {
    SPDLOG_LOGGER_ERROR(mLog, "Component or node with name {} not found", comp);
    throw InvalidArgumentException();
  }
}

void Simulation::logIdObjAttribute(const String &comp, const String &attr) {
  CPS::AttributeBase::Ptr attrPtr = getIdObjAttribute(comp, attr);
  String name = comp + "." + attr;
  logAttribute(name, attrPtr);
}

void Simulation::logAttribute(String name, CPS::AttributeBase::Ptr attr) {
  if (mLoggers.size() > 0) {
    mLoggers[0]->logAttribute(name, attr);
  } else {
    throw SystemError("Cannot log attributes when no logger is configured for "
                      "this simulation!");
  }
}
