// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "../Examples.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_GFM.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_ImpedanceLoad.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_PiLine.h>

using namespace DPsim;
using namespace CPS;

class TimedDataLogger final : public DataLogger {
public:
  explicit TimedDataLogger(const String &name) : DataLogger(name) {}

  void stop() override {
    const auto start = std::chrono::steady_clock::now();
    DataLogger::stop();
    mStopSeconds =
        std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
            .count();
  }

  Real stopSeconds() const { return mStopSeconds; }

private:
  Real mStopSeconds = 0.0;
};

class TimedRealTimeDataLogger final : public RealTimeDataLogger {
public:
  TimedRealTimeDataLogger(std::filesystem::path filename, Real finalTime,
                          Real timeStep)
      : RealTimeDataLogger(filename, finalTime, timeStep) {}

  void stop() override {
    const auto start = std::chrono::steady_clock::now();
    RealTimeDataLogger::stop();
    mStopSeconds =
        std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
            .count();
  }

  Real stopSeconds() const { return mStopSeconds; }

private:
  Real mStopSeconds = 0.0;
};

/// Record the actual scheduler task wall time on every simulated time step.
/// The columns retain the DPsim task names so pre-step, solve/integration, and
/// post-step work can be distinguished without timing an external surrogate.
class PerStepTimingScheduler final : public Scheduler {
public:
  explicit PerStepTimingScheduler(
      String filename, std::shared_ptr<EMT::Ph3::SSN_GFM> newtonGfm = nullptr)
      : Scheduler(Logger::Level::off), mFilename(std::move(filename)),
        mComponentFilename(mFilename.substr(0, mFilename.rfind('/')) +
                           "/per_step_component_timing.csv"),
        mNewtonGfm(std::move(newtonGfm)) {}

  void createSchedule(const Task::List &tasks, const Edges &inEdges,
                      const Edges &outEdges) override {
    mTasks = tasks;
    mInEdges = inEdges;
    mOutEdges = outEdges;
    resolveDeps(mTasks, mInEdges, mOutEdges);
    topologicalSort(mTasks, mInEdges, mOutEdges, mSchedule);
    mColumns.reserve(mSchedule.size());
    for (const auto &task : mSchedule) {
      String name = task->toString();
      std::replace(name.begin(), name.end(), ',', '_');
      mColumns.push_back(std::move(name));
    }
    if (mNewtonGfm) {
      for (std::size_t i = 0; i < mColumns.size(); ++i) {
        const String &name = mColumns[i];
        if (name.find(".MnaPreStep") != String::npos) {
          (name.find("GFM.MnaPreStep") != String::npos ? mGfmPreTasks
                                                       : mOtherPreTasks)
              .push_back(i);
        } else if (name.find(".Solve") != String::npos) {
          mSolveTasks.push_back(i);
        } else if (name.find(".MnaPostStep") != String::npos) {
          (name.find("GFM.MnaPostStep") != String::npos ? mGfmPostTasks
                                                        : mOtherPostTasks)
              .push_back(i);
        } else if (name.find(".Write") != String::npos) {
          mLoggerTasks.push_back(i);
        } else {
          mOtherTasks.push_back(i);
        }
      }
      if (mGfmPreTasks.size() != 1 || mGfmPostTasks.size() != 1 ||
          mSolveTasks.size() != 1)
        throw std::runtime_error("Newton scheduler expected one GFM pre/post "
                                 "task and one MNA solve.");
    }
  }

  void step(Real time, Int timeStepCount) override {
    std::vector<Real> elapsed(mSchedule.size(), 0.0);
    if (!mNewtonGfm) {
      for (std::size_t i = 0; i < mSchedule.size(); ++i)
        executeMeasured(i, time, timeStepCount, elapsed);
    } else {
      // Fixed network components prepare their histories once. The GFM tangent
      // and MNA system are rebuilt and solved on each Newton correction.
      for (std::size_t i : mOtherTasks)
        executeMeasured(i, time, timeStepCount, elapsed);
      for (std::size_t i : mOtherPreTasks)
        executeMeasured(i, time, timeStepCount, elapsed);

      mNewtonGfm->beginImplicitNewtonStep();
      Matrix guessState = mNewtonGfm->getState();
      Matrix guessVoltage = mNewtonGfm->getInterfaceVoltage();
      Real residual = std::numeric_limits<Real>::infinity();
      Bool converged = false;
      UInt iterations = 0;
      for (; iterations < mNewtonMaxIterations; ++iterations) {
        mNewtonGfm->setImplicitNewtonGuess(guessState, guessVoltage);
        mNewtonGfm->restoreImplicitNewtonBase();
        executeMeasured(mGfmPreTasks.front(), time, timeStepCount, elapsed);
        executeMeasured(mSolveTasks.front(), time, timeStepCount, elapsed);
        executeMeasured(mGfmPostTasks.front(), time, timeStepCount, elapsed);

        residual = mNewtonGfm->implicitNewtonResidual();
        if (residual <= mNewtonTolerance) {
          converged = true;
          ++iterations;
          break;
        }
        guessState = mNewtonGfm->getState();
        guessVoltage = mNewtonGfm->getInterfaceVoltage();
      }
      mNewtonGfm->endImplicitNewtonStep();
      if (!converged)
        throw std::runtime_error(
            "GFM/network Newton iteration did not converge at t=" +
            std::to_string(time) +
            " s; scaled residual=" + std::to_string(residual));

      for (std::size_t i : mOtherPostTasks)
        executeMeasured(i, time, timeStepCount, elapsed);
      for (std::size_t i : mLoggerTasks)
        executeMeasured(i, time, timeStepCount, elapsed);
      mStepIterations.push_back(iterations);
      mStepResiduals.push_back(residual);
    }
    mTimes.push_back(time);
    mSamples.push_back(std::move(elapsed));
  }

  void stop() override {
    std::ofstream out(mFilename);
    out << "simulation_time_s";
    for (const auto &name : mColumns)
      out << ',' << name << "_wall_time_s";
    out << '\n' << std::setprecision(12);
    for (std::size_t row = 0; row < mTimes.size(); ++row) {
      out << mTimes[row];
      for (Real value : mSamples[row])
        out << ',' << value;
      out << '\n';
    }

    if (mNewtonGfm) {
      const String iterationFilename =
          mFilename.substr(0, mFilename.rfind('/')) +
          "/newton_iteration_timing.csv";
      std::ofstream iterationOut(iterationFilename);
      iterationOut << "simulation_time_s,newton_iterations,scaled_residual\n"
                   << std::setprecision(12);
      for (std::size_t row = 0; row < mTimes.size(); ++row)
        iterationOut << mTimes[row] << ',' << mStepIterations[row] << ','
                     << mStepResiduals[row] << '\n';
    }

    std::ofstream componentOut(mComponentFilename);
    componentOut
        << "simulation_time_s,GFM_pre_step_s,GFM_post_step_s,GFM_total_s,"
           "PiLine_pre_step_s,PiLine_post_step_s,PiLine_total_s,"
           "BaseLoad_pre_step_s,BaseLoad_post_step_s,BaseLoad_total_s,"
           "StepLoad_pre_step_s,StepLoad_post_step_s,StepLoad_total_s,"
           "Loads_total_s,Other_component_tasks_s,Solver_task_s,Data_logger_"
           "task_s\n"
        << std::setprecision(12);
    for (std::size_t row = 0; row < mTimes.size(); ++row) {
      std::map<String, Real> pre, post;
      Real solver = 0.0;
      Real logging = 0.0;
      Real other = 0.0;
      for (std::size_t col = 0; col < mColumns.size(); ++col) {
        const String &name = mColumns[col];
        const auto prePos = name.find(".MnaPreStep");
        const auto postPos = name.find(".MnaPostStep");
        if (prePos != String::npos) {
          pre[name.substr(0, prePos)] += mSamples[row][col];
        } else if (postPos != String::npos) {
          post[name.substr(0, postPos)] += mSamples[row][col];
        } else if (name.find(".Solve") != String::npos) {
          solver += mSamples[row][col];
        } else if (name.find(".Write") != String::npos) {
          logging += mSamples[row][col];
        } else {
          other += mSamples[row][col];
        }
      }
      const Real gfm = pre["GFM"] + post["GFM"];
      const Real piLine = pre["PiLine"] + post["PiLine"];
      const Real baseLoad = pre["BaseLoad"] + post["BaseLoad"];
      const Real stepLoad = pre["StepLoad"] + post["StepLoad"];
      for (const auto &entry : pre)
        if (entry.first != "GFM" && entry.first != "PiLine" &&
            entry.first != "BaseLoad" && entry.first != "StepLoad")
          other += entry.second;
      for (const auto &entry : post)
        if (entry.first != "GFM" && entry.first != "PiLine" &&
            entry.first != "BaseLoad" && entry.first != "StepLoad")
          other += entry.second;
      componentOut << mTimes[row] << ',' << pre["GFM"] << ',' << post["GFM"]
                   << ',' << gfm << ',' << pre["PiLine"] << ','
                   << post["PiLine"] << ',' << piLine << ',' << pre["BaseLoad"]
                   << ',' << post["BaseLoad"] << ',' << baseLoad << ','
                   << pre["StepLoad"] << ',' << post["StepLoad"] << ','
                   << stepLoad << ',' << baseLoad + stepLoad << ',' << other
                   << ',' << solver << ',' << logging << '\n';
    }
  }

  std::size_t sampleCount() const { return mTimes.size(); }
  UInt maximumNewtonIterations() const {
    return mStepIterations.empty() ? 0
                                   : *std::max_element(mStepIterations.begin(),
                                                       mStepIterations.end());
  }
  Real meanNewtonIterations() const {
    if (mStepIterations.empty())
      return 0.0;
    Real total = 0.0;
    for (UInt value : mStepIterations)
      total += value;
    return total / static_cast<Real>(mStepIterations.size());
  }
  Real maximumNewtonResidual() const {
    return mStepResiduals.empty() ? 0.0
                                  : *std::max_element(mStepResiduals.begin(),
                                                      mStepResiduals.end());
  }

private:
  void executeMeasured(std::size_t index, Real time, Int timeStepCount,
                       std::vector<Real> &elapsed) {
    const auto start = std::chrono::steady_clock::now();
    mSchedule[index]->execute(time, timeStepCount);
    elapsed[index] +=
        std::chrono::duration<Real>(std::chrono::steady_clock::now() - start)
            .count();
  }

  static constexpr UInt mNewtonMaxIterations = 12;
  static constexpr Real mNewtonTolerance = 1.0;
  String mFilename;
  String mComponentFilename;
  std::shared_ptr<EMT::Ph3::SSN_GFM> mNewtonGfm;
  Task::List mTasks;
  Edges mInEdges, mOutEdges;
  Task::List mSchedule;
  std::vector<String> mColumns;
  std::vector<Real> mTimes;
  std::vector<std::vector<Real>> mSamples;
  std::vector<std::size_t> mGfmPreTasks, mGfmPostTasks, mOtherPreTasks,
      mOtherPostTasks, mSolveTasks, mLoggerTasks, mOtherTasks;
  std::vector<UInt> mStepIterations;
  std::vector<Real> mStepResiduals;
};

static String trimCsvField(String value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == String::npos)
    return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

static std::vector<String> splitCsvLine(const String &line) {
  std::vector<String> fields;
  std::stringstream stream(line);
  String field;
  while (std::getline(stream, field, ','))
    fields.push_back(field);
  if (!line.empty() && line.back() == ',')
    fields.emplace_back();
  return fields;
}

/// Put the time column first, then all GFM columns, then all remaining columns.
static void reorderGfmCsvColumns(const String &path) {
  std::ifstream input(path);
  String line;
  if (!std::getline(input, line))
    throw std::runtime_error("Could not read measurement CSV header: " + path);
  const auto headers = splitCsvLine(line);
  std::vector<std::size_t> order;
  const auto time =
      std::find_if(headers.begin(), headers.end(),
                   [](const auto &h) { return trimCsvField(h) == "time"; });
  if (time == headers.end())
    throw std::runtime_error("Measurement CSV is missing its time column.");
  order.push_back(static_cast<std::size_t>(time - headers.begin()));
  for (std::size_t index = 0; index < headers.size(); ++index)
    if (index != order.front() &&
        trimCsvField(headers[index]).rfind("GFM_", 0) == 0)
      order.push_back(index);
  for (std::size_t index = 0; index < headers.size(); ++index)
    if (index != order.front() &&
        trimCsvField(headers[index]).rfind("GFM_", 0) != 0)
      order.push_back(index);

  const String temporaryPath = path + ".ordered";
  std::ofstream output(temporaryPath);
  for (std::size_t index = 0; index < order.size(); ++index) {
    if (index)
      output << ',';
    output << trimCsvField(headers[order[index]]);
  }
  output << '\n';
  while (std::getline(input, line)) {
    const auto fields = splitCsvLine(line);
    if (fields.size() != headers.size())
      throw std::runtime_error("Unexpected field count in measurement CSV.");
    for (std::size_t index = 0; index < order.size(); ++index) {
      if (index)
        output << ',';
      output << fields[order[index]];
    }
    output << '\n';
  }
  input.close();
  output.close();
  if (std::remove(path.c_str()) != 0 ||
      std::rename(temporaryPath.c_str(), path.c_str()) != 0)
    throw std::runtime_error(
        "Could not replace measurement CSV with sorted columns.");
}

int main(int argc, char *argv[]) {
  Bool useCsvLogger = false;
  Bool useNewtonIterations = false;
  for (int arg = 1; arg < argc; ++arg) {
    const String option(argv[arg]);
    if (option == "--csv-logger")
      useCsvLogger = true;
    else if (option == "--newton")
      useNewtonIterations = true;
    else
      throw std::invalid_argument("Unknown option: " + option);
  }
  const Bool useRealtimeLogger = !useCsvLogger;
  // -------------------------------------------------------------------------
  // Simulation configuration
  // -------------------------------------------------------------------------
  const Real finalTime = 3;
  const Real timeStep = 50e-6;
  const String simName = "EMT_SSN_GFM_LoadStep";

  // -------------------------------------------------------------------------
  // Electrical parameters
  //
  // In the paper Ug = 220 V is consistent with phase-to-neutral RMS voltage:
  // the plotted steady d-axis voltage is approximately sqrt(2) * 220 V.
  // DPsim network/load components use line-to-line RMS voltage, whereas the
  // GFM dq voltage reference uses phase peak voltage.
  // -------------------------------------------------------------------------
  const Real phaseVoltageRms = 220.0;
  const Real lineToLineVoltageRms = std::sqrt(3.0) * phaseVoltageRms;
  const Real phaseVoltagePeak = std::sqrt(2.0) * phaseVoltageRms;

  const Real nominalFrequency = 50.0;
  const Real nominalOmega = 2.0 * PI * nominalFrequency;

  // Paper values
  const Real filterInductance = 3e-3;
  const Real filterCapacitance = 20e-6;
  const Real virtualInertia = 0.2;
  const Real dampingCoefficient = 25.0;
  const Real voltageDroopGain = 1.0 / 15.0;
  const Real voltageLoopBandwidth = 200.0;
  const Real currentLoopBandwidth = 1683.0;
  const Real powerFilterCutoff = 100.0;
  const Real samplingFrequency = 20e3;

  // The paper does not explicitly specify Rf or the algebraic coupling
  // resistance Rc used by this implementation.
  const Real filterResistance = 0.05;
  const Real internalCouplingResistance = 0.05;

  // Feeder connection represented by the same series R-L in PF and EMT.
  // Small shunt capacitance completes the PiLine model.
  const Real connectionResistance = 0.20;
  const Real connectionInductance = 1e-4;
  const Real connectionCapacitance = 1e-7;

  // -------------------------------------------------------------------------
  // Controller gains
  // -------------------------------------------------------------------------
  const Real voltageLoopDamping = 1.0 / std::sqrt(2.0);

  const Real kpVoltage =
      2.0 * voltageLoopDamping * voltageLoopBandwidth * filterCapacitance;
  const Real kiVoltage =
      voltageLoopBandwidth * voltageLoopBandwidth * filterCapacitance;

  const Real kpCurrent = currentLoopBandwidth * filterInductance;
  const Real kiCurrent = currentLoopBandwidth * filterResistance;

  // The paper states Kq = 300 while its reactive power signals are presented
  // in kvar. The C++ model evaluates Q in var, therefore convert the gain:
  // 300 per kvar -> 0.3 per var.
  const Real reactiveIntegralGain = 300.0 / 1000.0;

  // The paper states Kc = 15. Keep it disabled for the first stable baseline
  // because the sign depends on the precise capacitor-current convention.
  // After the baseline is verified, change this to 15.0 and validate damping.
  const Real activeDampingGain = 0.0;

  // First-order approximation of roughly 1.5 sampling periods of delay.
  const Real delayBandwidth = samplingFrequency / 1.5;

  // -------------------------------------------------------------------------
  // Operating point and disturbance
  // -------------------------------------------------------------------------
  const Real baseLoadActivePower = 12e3;
  const Real baseLoadReactivePower = 0.0;

  const Real stepLoadActivePower = 3e3;
  const Real stepLoadReactivePower = 0.0;

  // The initial active-power reference must match the initial islanded load.
  const Real activePowerReference = baseLoadActivePower;
  // Include the feeder's initial reactive demand in the islanded reference.
  // With a series inductance, Q at the converter is not zero even though the
  // base load is resistive; leaving Qref at zero makes the integral excitation
  // controller continuously reduce its voltage magnitude.
  const Real baseLoadCurrentRms =
      std::sqrt(baseLoadActivePower * baseLoadActivePower +
                baseLoadReactivePower * baseLoadReactivePower) /
      (std::sqrt(3.0) * lineToLineVoltageRms);
  const Real feederReactivePower = 3.0 * baseLoadCurrentRms *
                                       baseLoadCurrentRms * nominalOmega *
                                       connectionInductance -
                                   3.0 * phaseVoltageRms * phaseVoltageRms *
                                       nominalOmega * connectionCapacitance;
  const Real reactivePowerReference =
      baseLoadReactivePower + feederReactivePower;

  const Real loadStepStartTime = 1.0;
  const Real loadStepEndTime = 2.0;

  // =========================================================================
  // Power-flow initialization
  // =========================================================================
  const String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto nGfmPF = SimNode<Complex>::make("nGfm", PhaseType::Single);
  auto nLoadPF = SimNode<Complex>::make("nLoad", PhaseType::Single);

  auto extnetPF =
      SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
  extnetPF->setParameters(lineToLineVoltageRms);
  extnetPF->setBaseVoltage(lineToLineVoltageRms);
  extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
  linePF->setParameters(connectionResistance, connectionInductance,
                        connectionCapacitance);
  linePF->setBaseVoltage(lineToLineVoltageRms);

  auto loadPF = SP::Ph1::Load::make("BaseLoad", Logger::Level::debug);
  loadPF->setParameters(baseLoadActivePower, baseLoadReactivePower,
                        lineToLineVoltageRms);

  extnetPF->connect({nGfmPF});
  linePF->connect({nGfmPF, nLoadPF});
  loadPF->connect({nLoadPF});

  auto systemPF =
      SystemTopology(nominalFrequency, SystemNodeList{nGfmPF, nLoadPF},
                     SystemComponentList{extnetPF, linePF, loadPF});

  auto loggerPF = DataLogger::make(simNamePF);
  loggerPF->logAttribute("Voltage_GFM", nGfmPF->attribute("v"));
  loggerPF->logAttribute("Voltage_Load", nLoadPF->attribute("v"));

  Simulation simPF(simNamePF, Logger::Level::debug);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(1.0);
  simPF.setFinalTime(2.0);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(false);
  simPF.addLogger(loggerPF);
  simPF.run();

  // =========================================================================
  // Dynamic EMT simulation
  // =========================================================================
  const String simNameEMT = simName +
                            (useCsvLogger ? "_EMT_CSVLogger" : "_EMT") +
                            (useNewtonIterations ? "_Newton" : "");
  Logger::setLogDir("logs/" + simNameEMT);
  std::filesystem::create_directories("logs/" + simNameEMT);

  auto nGfmEMT = SimNode<Real>::make("nGfm", PhaseType::ABC);
  auto nLoadEMT = SimNode<Real>::make("nLoad", PhaseType::ABC);
  auto nStepLoadEMT = SimNode<Real>::make("nStepLoad", PhaseType::ABC);

  auto gfm = EMT::Ph3::SSN_GFM::make("GFM", "GFM", Logger::Level::debug);

  gfm->setParameters(filterInductance, filterCapacitance, filterResistance,
                     internalCouplingResistance, phaseVoltagePeak, nominalOmega,
                     activePowerReference, reactivePowerReference,
                     virtualInertia, dampingCoefficient, voltageDroopGain,
                     reactiveIntegralGain, kpVoltage, kiVoltage, kpCurrent,
                     kiCurrent, activeDampingGain, powerFilterCutoff,
                     delayBandwidth);

  gfm->setNumericalLinearizationParameters(1e-6, 1e-8);

  auto piLine = EMT::Ph3::SSN::PiLine::make("PiLine", Logger::Level::debug);
  piLine->setParameters(
      Math::singlePhaseParameterToThreePhase(connectionResistance),
      Math::singlePhaseParameterToThreePhase(connectionInductance),
      Math::singlePhaseParameterToThreePhase(connectionCapacitance));

  auto baseLoad =
      EMT::Ph3::SSN::ImpedanceLoad::make("BaseLoad", Logger::Level::debug);
  baseLoad->setParameters(baseLoadActivePower, baseLoadReactivePower,
                          lineToLineVoltageRms);

  auto stepLoad =
      EMT::Ph3::SSN::ImpedanceLoad::make("StepLoad", Logger::Level::debug);
  stepLoad->setParameters(stepLoadActivePower, stepLoadReactivePower,
                          lineToLineVoltageRms);

  const Real switchOpenResistance = 1e9;
  const Real switchClosedResistance = 1e-6;

  auto loadSwitch = EMT::Ph3::Switch::make("LoadSwitch", Logger::Level::debug);
  loadSwitch->setParameters(
      Math::singlePhaseParameterToThreePhase(switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(switchClosedResistance));
  loadSwitch->openSwitch();

  // The inherited interface convention is:
  // v_intf = v_terminal1 - v_terminal0.
  gfm->connect({SimNode<Real>::GND, nGfmEMT});
  piLine->connect({nGfmEMT, nLoadEMT});
  baseLoad->connect({nLoadEMT});
  loadSwitch->connect({nLoadEMT, nStepLoadEMT});
  stepLoad->connect({nStepLoadEMT});

  auto systemEMT = SystemTopology(
      nominalFrequency, SystemNodeList{nGfmEMT, nLoadEMT, nStepLoadEMT},
      SystemComponentList{gfm, piLine, baseLoad, loadSwitch, stepLoad});

  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  // Keep the terminal operating-point metadata consistent with the initial
  // power-flow solution and the GFM reference.
  gfm->terminal(1)->setPower(
      Complex(baseLoadActivePower, reactivePowerReference));

  // -------------------------------------------------------------------------
  // Logging
  // -------------------------------------------------------------------------
  std::shared_ptr<TimedDataLogger> csvLogger;
  std::shared_ptr<TimedRealTimeDataLogger> realtimeLogger;
  DataLoggerInterface::Ptr loggerEMT;
  if (useRealtimeLogger) {
    const std::filesystem::path filename =
        "logs/" + simNameEMT + "/" + simNameEMT + ".csv";
    realtimeLogger = std::make_shared<TimedRealTimeDataLogger>(
        filename, finalTime, timeStep);
    loggerEMT = realtimeLogger;
  } else {
    csvLogger = std::make_shared<TimedDataLogger>(simNameEMT);
    loggerEMT = csvLogger;
  }

  loggerEMT->logAttribute("GFM_BusVoltage_V", nGfmEMT->attribute("v"));

  loggerEMT->logAttribute("GFM_InterfaceVoltage_V", gfm->attribute("v_intf"));
  loggerEMT->logAttribute("GFM_InterfaceCurrent_A", gfm->attribute("i_intf"));
  loggerEMT->logAttribute("GFM_ActivePower_W", gfm->attribute("p_inst"));
  loggerEMT->logAttribute("GFM_ReactivePower_var", gfm->attribute("q_inst"));
  loggerEMT->logAttribute("GFM_ElectricalFrequency_rad_per_s",
                          gfm->attribute("omega_gfm"));
  loggerEMT->logAttribute("GFM_ElectricalAngle_rad",
                          gfm->attribute("theta_gfm"));
  loggerEMT->logAttribute("GFM_VoltageMagnitude_V",
                          gfm->attribute("voltage_magnitude_gfm"));
  loggerEMT->logAttribute("GFM_CapacitorVoltage_d_V", gfm->attribute("vc_d"));
  loggerEMT->logAttribute("GFM_CapacitorVoltage_q_V", gfm->attribute("vc_q"));
  loggerEMT->logAttribute("GFM_GridCurrent_d_A", gfm->attribute("i_grid_d"));
  loggerEMT->logAttribute("GFM_GridCurrent_q_A", gfm->attribute("i_grid_q"));
  loggerEMT->logAttribute("GFM_FilterCurrent_d_A", gfm->attribute("if_d"));
  loggerEMT->logAttribute("GFM_FilterCurrent_q_A", gfm->attribute("if_q"));
  loggerEMT->logAttribute("GFM_VoltageReferenceD_V", gfm->attribute("v_ref_d"));
  loggerEMT->logAttribute("GFM_VoltageReferenceQ_V", gfm->attribute("v_ref_q"));
  const std::vector<String> stateAttributeNames = {
      "state_p_filtered_W",
      "state_q_filtered_var",
      "state_omega_rad_per_s",
      "state_theta_rad",
      "state_voltage_magnitude_V",
      "state_voltage_integrator_d_Vs",
      "state_voltage_integrator_q_Vs",
      "state_current_integrator_d_As",
      "state_current_integrator_q_As",
      "state_delay_voltage_d_V",
      "state_delay_voltage_q_V",
      "state_capacitor_voltage_a_V",
      "state_capacitor_voltage_b_V",
      "state_capacitor_voltage_c_V",
      "state_filter_current_a_A",
      "state_filter_current_b_A",
      "state_filter_current_c_A"};
  for (const auto &stateName : stateAttributeNames)
    loggerEMT->logAttribute("GFM_" + stateName, gfm->attribute(stateName));
  loggerEMT->logAttribute("GFM_ActivePowerReference_W",
                          AttributeStatic<Real>::make(activePowerReference));
  loggerEMT->logAttribute("GFM_ReactivePowerReference_var",
                          AttributeStatic<Real>::make(reactivePowerReference));
  loggerEMT->logAttribute("GFM_VirtualResistance_ohm",
                          AttributeStatic<Real>::make(0.0));
  loggerEMT->logAttribute("GFM_VirtualReactance_ohm",
                          AttributeStatic<Real>::make(0.0));

  loggerEMT->logAttribute("PiLine_InterfaceVoltage_V",
                          piLine->attribute("v_intf"));
  loggerEMT->logAttribute("PiLine_SeriesCurrent_A",
                          piLine->attribute("i_intf"));
  loggerEMT->logAttribute("PiLine_R_series_per_phase_ohm",
                          AttributeStatic<Real>::make(connectionResistance));
  loggerEMT->logAttribute("PiLine_L_series_per_phase_H",
                          AttributeStatic<Real>::make(connectionInductance));
  loggerEMT->logAttribute("PiLine_C_shunt_per_phase_F",
                          AttributeStatic<Real>::make(connectionCapacitance));

  loggerEMT->logAttribute("BaseLoad_Voltage_V", nLoadEMT->attribute("v"));
  loggerEMT->logAttribute("BaseLoad_Current_A", baseLoad->attribute("i_intf"));
  loggerEMT->logAttribute("BaseLoad_ActivePower_W",
                          AttributeStatic<Real>::make(baseLoadActivePower));
  loggerEMT->logAttribute("BaseLoad_ReactivePower_var",
                          AttributeStatic<Real>::make(baseLoadReactivePower));
  loggerEMT->logAttribute("StepLoad_Voltage_V", nStepLoadEMT->attribute("v"));
  loggerEMT->logAttribute("StepLoad_Current_A", stepLoad->attribute("i_intf"));
  loggerEMT->logAttribute("StepLoad_ActivePower_W",
                          AttributeStatic<Real>::make(stepLoadActivePower));
  loggerEMT->logAttribute("StepLoad_ReactivePower_var",
                          AttributeStatic<Real>::make(stepLoadReactivePower));

  // RXLoad is implemented as a voltage-dependent equivalent impedance.
  // Log its nominal per-phase R+jX calculated from P+jQ at V_LL.
  const Real baseLoadResistanceOhm =
      lineToLineVoltageRms * lineToLineVoltageRms * baseLoadActivePower /
      (baseLoadActivePower * baseLoadActivePower +
       baseLoadReactivePower * baseLoadReactivePower);
  const Real baseLoadReactanceOhm =
      lineToLineVoltageRms * lineToLineVoltageRms * baseLoadReactivePower /
      (baseLoadActivePower * baseLoadActivePower +
       baseLoadReactivePower * baseLoadReactivePower);
  const Real stepLoadResistanceOhm =
      lineToLineVoltageRms * lineToLineVoltageRms * stepLoadActivePower /
      (stepLoadActivePower * stepLoadActivePower +
       stepLoadReactivePower * stepLoadReactivePower);
  const Real stepLoadReactanceOhm =
      lineToLineVoltageRms * lineToLineVoltageRms * stepLoadReactivePower /
      (stepLoadActivePower * stepLoadActivePower +
       stepLoadReactivePower * stepLoadReactivePower);
  loggerEMT->logAttribute("BaseLoad_R_equivalent_per_phase_ohm",
                          AttributeStatic<Real>::make(baseLoadResistanceOhm));
  loggerEMT->logAttribute("BaseLoad_X_equivalent_per_phase_ohm",
                          AttributeStatic<Real>::make(baseLoadReactanceOhm));
  loggerEMT->logAttribute("StepLoad_R_equivalent_per_phase_ohm",
                          AttributeStatic<Real>::make(stepLoadResistanceOhm));
  loggerEMT->logAttribute("StepLoad_X_equivalent_per_phase_ohm",
                          AttributeStatic<Real>::make(stepLoadReactanceOhm));

  // -------------------------------------------------------------------------
  // Events and simulation
  // -------------------------------------------------------------------------
  Simulation sim(simNameEMT, Logger::Level::debug);
  auto timingScheduler = std::make_shared<PerStepTimingScheduler>(
      "logs/" + simNameEMT + "/per_step_task_timing.csv",
      useNewtonIterations ? gfm : nullptr);
  sim.setScheduler(timingScheduler);

  sim.addEvent(SwitchEvent3Ph::make(loadStepStartTime, loadSwitch, true));
  sim.addEvent(SwitchEvent3Ph::make(loadStepEndTime, loadSwitch, false));

  sim.setSystem(systemEMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.doSystemMatrixRecomputation(true);
  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(loggerEMT);
  const auto wallStart = std::chrono::steady_clock::now();
  sim.run();
  const auto wallEnd = std::chrono::steady_clock::now();
  const String measurementCsvPath =
      useRealtimeLogger ? "logs/" + simNameEMT + "/" + simNameEMT + ".csv"
                        : "logs/" + simNameEMT + ".csv";
  reorderGfmCsvColumns(measurementCsvPath);
  // DPsim reports cumulative/average/max LU re-factorization and linear-solve
  // timings here. The per-step CSV above separately records every pre-step,
  // complete solver/integration, and post-step task.
  sim.logLUTimes();
  const Real wallSeconds =
      std::chrono::duration<Real>(wallEnd - wallStart).count();
  const Real loggerStopSeconds = useRealtimeLogger
                                     ? realtimeLogger->stopSeconds()
                                     : csvLogger->stopSeconds();
  std::ofstream performance("logs/" + simNameEMT + "/performance_summary.csv");
  performance << "metric,value,unit\n"
              << "simulation_wall_clock," << std::setprecision(12)
              << wallSeconds << ",s\n"
              << "simulated_duration," << finalTime << ",s\n"
              << "time_step," << timeStep << ",s\n"
              << "number_of_executed_steps," << timingScheduler->sampleCount()
              << ",steps\n"
              << "data_logger," << (useRealtimeLogger ? "realtime" : "csv")
              << ",type\n"
              << "data_logger_stop_flush," << loggerStopSeconds << ",s\n";
  if (useNewtonIterations) {
    performance << "integration_method,fully_implicit_newton,type\n"
                << "newton_scaled_tolerance,1,scaled_residual\n"
                << "newton_max_iterations,12,iterations\n"
                << "newton_mean_iterations_per_step,"
                << timingScheduler->meanNewtonIterations() << ",iterations\n"
                << "newton_max_iterations_observed,"
                << timingScheduler->maximumNewtonIterations() << ",iterations\n"
                << "newton_max_final_scaled_residual,"
                << timingScheduler->maximumNewtonResidual()
                << ",scaled_residual\n";
  } else {
    performance << "integration_method,one_local_linearization,type\n";
  }

  return 0;
}
