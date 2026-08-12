// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.h>
#include <dpsim-models/EMT/EMT_Ph3_GFL.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_GFL.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_GFL_Split.h>
#include <dpsim-models/EMT/EMT_Ph3_SeriesResistor.h>
#include <dpsim-models/EMT/EMT_Ph3_SeriesSwitch.h>

using namespace DPsim;
using namespace CPS;

/// \brief Four-model EMT benchmark for grid-following converter implementations.
///
/// All four cases use the same external grid:
///
///   GFL -- R_grid -- L_grid -- NetworkInjection
///
/// Execution order:
///
///   1) EMT::Ph3::GFL
///      - new composite EMT GFL
///      - controlled voltage source + Rf + Lf + shunt Cf
///      - no Rc, no transformer
///      - fixed global MNA matrix
///
///   2) EMT::Ph3::SSN_GFL
///      - monolithic 14-state locally-linearized variable SSN model
///
///   3) EMT::Ph3::SSN_GFL_Split
///      - split controller + fixed electrical SSN plant
///
///   4) EMT::Ph3::AvVoltSourceInverterStateSpace
///      - previous 14-state affine state-space implementation
///
/// All four cases use the same PF-initialized nGrid/nSeries/nPcc topology.
///
/// Disturbance:
///
///   PCC -- breaker -- nLoad -- Rload -- GND
///
/// The breaker is open initially and closes at t = 5 s. Rload is selected
/// such that the balanced three-phase resistive load consumes 5 kW at the
/// pre-disturbance PCC RMS voltage.
///
/// The SeriesSwitch is a discrete MNA switch. With matrix recomputation
/// disabled, DPsim uses precomputed matrices for its open/closed states, so
/// the fixed-matrix GFL cases remain fixed-matrix cases.
///
/// For the three Rc-based models, P/Q are referenced at the capacitor side.
/// The new EMT::Ph3::GFL has no Rc and regulates PCC power directly, therefore
/// it uses the equivalent PCC operating point from the same power flow.
///
/// Common logged signals:
///   v_grid, v_series, v_pcc, i_inv,
///   v_load, i_breaker, i_load,
///   vc_d, vc_q, igrid_d, igrid_q, omega_pll
///
/// Wall time measures sim.run() only.
class Example_GFL_Four_Model_BreakerLoadStep {
public:
  Example_GFL_Four_Model_BreakerLoadStep()
      : mTimeStepEMT(100e-6), mFinalTimeEMT(10.0), mSystemFrequency(50.0),
        mSystemOmega(2.0 * PI * mSystemFrequency),
        mGridVoltageRMSLineToLine(400.0), mGridResistance(0.3),
        mGridInductance(0.1e-3), mLf(2e-3), mCf(10e-6), mRf(0.2), mRc(0.2),
        mKpPLL(0.25), mKiPLL(0.2), mOmegaCutoff(mSystemOmega),
        mPrefFilter(10000.0), mQrefFilter(5000.0), mKpPowerCtrl(0.05),
        mKiPowerCtrl(0.2), mKpCurrCtrl(0.25), mKiCurrCtrl(1.0),
        mLoadStepTime(5.0), mLoadStepP(5000.0), mBreakerOpenResistance(1e9),
        mBreakerClosedResistance(1e-3) {}

  void run() const {
    const String baseName = "EMT_Ph3_GFL_Four_Model_BreakerLoadStep";

    const auto [pPccRef, qPccRef] =
        pccPowerFromFilterPowerReference(mPrefFilter, mQrefFilter);

    printConfiguration(pPccRef, qPccRef);

    const auto systemPF = runPowerFlow(baseName + "_PF", pPccRef, qPccRef);

    std::vector<TimingResult> timings;
    timings.reserve(4);

    // New composite GFL runs first.
    timings.push_back(runGFL(baseName + "_GFL", systemPF, pPccRef, qPccRef));

    timings.push_back(runSSNGFL(baseName + "_SSN_GFL", systemPF));

    timings.push_back(runSSNGFLSplit(baseName + "_SSN_GFL_Split", systemPF));

    timings.push_back(runAvVoltStateSpace(
        baseName + "_AvVoltSourceInverterStateSpace", systemPF));

    printTimingSummary(timings);
  }

private:
  struct TimingResult {
    String label;
    String simulationName;
    double wallSeconds;
    Bool recomputeSystemMatrix;
  };

  // =========================================================================
  // POWER FLOW
  // =========================================================================
  SystemTopology runPowerFlow(const String &simName, Real pPccRef,
                              Real qPccRef) const {

    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Complex>::make("nGrid", PhaseType::Single);
    auto nSeries = SimNode<Complex>::make("nSeries", PhaseType::Single);
    auto nPcc = SimNode<Complex>::make("nPcc", PhaseType::Single);

    auto slack = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
    slack->setParameters(mGridVoltageRMSLineToLine);
    slack->setBaseVoltage(mGridVoltageRMSLineToLine);
    slack->modifyPowerFlowBusType(PowerflowBusType::VD);

    // Separate branches are intentional so nSeries exists in the PF and
    // receives a consistent initial voltage for the EMT R-L branch.
    auto resistorPF =
        SP::Ph1::PiLine::make("GridResistorPF", Logger::Level::off);
    resistorPF->setParameters(mGridResistance, 0.0, 0.0, 0.0);
    resistorPF->setBaseVoltage(mGridVoltageRMSLineToLine);

    auto inductorPF =
        SP::Ph1::PiLine::make("GridInductorPF", Logger::Level::off);
    inductorPF->setParameters(0.0, mGridInductance, 0.0, 0.0);
    inductorPF->setBaseVoltage(mGridVoltageRMSLineToLine);

    auto inverterInjection =
        SP::Ph1::Load::make("INV_GFL_PF", Logger::Level::off);
    inverterInjection->setParameters(-pPccRef, -qPccRef,
                                     mGridVoltageRMSLineToLine);
    inverterInjection->modifyPowerFlowBusType(PowerflowBusType::PQ);

    inverterInjection->connect({nPcc});
    resistorPF->connect({nPcc, nSeries});
    inductorPF->connect({nSeries, nGrid});
    slack->connect({nGrid});

    auto system = SystemTopology(
        mSystemFrequency, SystemNodeList{nGrid, nSeries, nPcc},
        SystemComponentList{slack, resistorPF, inductorPF, inverterInjection});

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_grid_pf", nGrid->attribute("v"));
    logger->logAttribute("v_series_pf", nSeries->attribute("v"));
    logger->logAttribute("v_pcc_pf", nPcc->attribute("v"));

    Simulation sim(simName, Logger::Level::off);
    sim.setSystem(system);
    sim.setTimeStep(1.0);
    sim.setFinalTime(2.0);
    sim.setDomain(Domain::SP);
    sim.setSolverType(Solver::Type::NRP);
    sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
    sim.doInitFromNodesAndTerminals(false);
    sim.addLogger(logger);
    sim.run();

    return system;
  }

  // =========================================================================
  // BREAKER-CONNECTED LOAD STEP
  // =========================================================================

  struct BreakerLoad {
    SimNode<Real>::Ptr node;
    std::shared_ptr<EMT::Ph3::SeriesSwitch> breaker;
    std::shared_ptr<EMT::Ph3::SeriesResistor> resistor;
  };

  BreakerLoad createBreakerLoad(const SimNode<Real>::Ptr &nPcc) const {

    BreakerLoad load;

    // The disconnected load node is physically tied to ground through Rload,
    // so zero initial voltage is correct while the breaker is open.
    load.node = SimNode<Real>::make("nLoad", PhaseType::ABC);

    load.breaker =
        EMT::Ph3::SeriesSwitch::make("LoadBreaker", Logger::Level::off);

    load.breaker->setParameters(mBreakerOpenResistance,
                                mBreakerClosedResistance);

    load.breaker->open();

    load.resistor =
        EMT::Ph3::SeriesResistor::make("LoadResistor", Logger::Level::off);

    // Temporary value. It is replaced after PF initialization with the value
    // computed from the actual pre-disturbance PCC RMS voltage.
    load.resistor->setParameters(mGridVoltageRMSLineToLine *
                                 mGridVoltageRMSLineToLine / mLoadStepP);

    // SeriesSwitch current is defined from terminal 1 to terminal 0.
    // This orientation makes positive breaker current flow PCC -> load.
    load.breaker->connect({load.node, nPcc});

    // Positive SeriesResistor current then flows nLoad -> GND.
    load.resistor->connect({EMT::SimNode::GND, load.node});

    return load;
  }

  Real calculateLoadResistance(Complex pccVoltageRMS) const {

    const Real vPccLLRMS = std::abs(pccVoltageRMS);

    if (vPccLLRMS < 1e-9)
      throw std::runtime_error(
          "Cannot calculate breaker-load resistance from zero PCC voltage.");

    // Balanced three-phase wye resistive load:
    //
    //   P_3ph = 3 * V_phase^2 / R_phase
    //         = V_LL^2 / R_phase
    //
    // hence R_phase = V_LL^2 / P_3ph.
    return vPccLLRMS * vPccLLRMS / mLoadStepP;
  }

  // =========================================================================
  // CASE 1: New EMT::Ph3::GFL
  // =========================================================================
  TimingResult runGFL(const String &simName, const SystemTopology &systemPF,
                      Real pPccRef, Real qPccRef) const {

    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Real>::make("nGrid", PhaseType::ABC);
    auto nSeries = SimNode<Real>::make("nSeries", PhaseType::ABC);
    auto nPcc = SimNode<Real>::make("nPcc", PhaseType::ABC);

    auto breakerLoad = createBreakerLoad(nPcc);

    auto slack = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::off);

    auto gridResistor =
        EMT::Ph3::Resistor::make("GridResistor", Logger::Level::off);
    gridResistor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridResistance));

    auto gridInductor =
        EMT::Ph3::Inductor::make("GridInductor", Logger::Level::off);
    gridInductor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridInductance));

    auto inverter = EMT::Ph3::GFL::make("INV_GFL", Logger::Level::off);

    // No Rc in this model: regulate the common PCC operating point directly.
    inverter->setParameters(mSystemOmega, mGridVoltageRMSLineToLine, pPccRef,
                            qPccRef);

    inverter->setControllerParameters(mKpPLL, mKiPLL, mKpPowerCtrl,
                                      mKiPowerCtrl, mKpCurrCtrl, mKiCurrCtrl,
                                      mOmegaCutoff);

    inverter->setFilterParameters(mLf, mCf, mRf);

    inverter->withControl(true);

    inverter->connect({nPcc});
    gridResistor->connect({nPcc, nSeries});
    gridInductor->connect({nSeries, nGrid});
    slack->connect({nGrid});

    auto system = SystemTopology(
        mSystemFrequency,
        SystemNodeList{nGrid, nSeries, nPcc, breakerLoad.node},
        SystemComponentList{slack, gridResistor, gridInductor, inverter,
                            breakerLoad.breaker, breakerLoad.resistor});

    system.initWithPowerflow(systemPF, Domain::EMT);

    const Real loadResistance =
        calculateLoadResistance(nPcc->initialSingleVoltage());

    breakerLoad.resistor->setParameters(loadResistance);

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_load", breakerLoad.node->attribute("v"));

    logger->logAttribute("i_breaker", breakerLoad.breaker->attribute("i_intf"));

    logger->logAttribute("i_load", breakerLoad.resistor->attribute("i_intf"));

    addNewGFLCommonSignals(logger, nGrid, nSeries, nPcc, inverter);

    return runTimedSimulation("GFL", simName, system, logger, false,
                              breakerLoad.breaker);
  }

  // =========================================================================
  // CASE 2: SSN_GFL
  // =========================================================================
  TimingResult runSSNGFL(const String &simName,
                         const SystemTopology &systemPF) const {

    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Real>::make("nGrid", PhaseType::ABC);
    auto nSeries = SimNode<Real>::make("nSeries", PhaseType::ABC);
    auto nPcc = SimNode<Real>::make("nPcc", PhaseType::ABC);

    auto breakerLoad = createBreakerLoad(nPcc);

    auto slack = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::off);

    auto gridResistor =
        EMT::Ph3::Resistor::make("GridResistor", Logger::Level::off);
    gridResistor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridResistance));

    auto gridInductor =
        EMT::Ph3::Inductor::make("GridInductor", Logger::Level::off);
    gridInductor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridInductance));

    auto inverter = EMT::Ph3::SSN_GFL::make("INV_GFL", Logger::Level::off);

    setStateSpaceGFLParameters(inverter);

    inverter->connect({EMT::SimNode::GND, nPcc});
    gridResistor->connect({nPcc, nSeries});
    gridInductor->connect({nSeries, nGrid});
    slack->connect({nGrid});

    auto system = SystemTopology(
        mSystemFrequency,
        SystemNodeList{nGrid, nSeries, nPcc, breakerLoad.node},
        SystemComponentList{slack, gridResistor, gridInductor, inverter,
                            breakerLoad.breaker, breakerLoad.resistor});

    system.initWithPowerflow(systemPF, Domain::EMT);

    const Real loadResistance =
        calculateLoadResistance(nPcc->initialSingleVoltage());

    breakerLoad.resistor->setParameters(loadResistance);

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_load", breakerLoad.node->attribute("v"));

    logger->logAttribute("i_breaker", breakerLoad.breaker->attribute("i_intf"));

    logger->logAttribute("i_load", breakerLoad.resistor->attribute("i_intf"));

    addStateSpaceCommonSignals(logger, nGrid, nSeries, nPcc, inverter);

    return runTimedSimulation("SSN_GFL", simName, system, logger, true,
                              breakerLoad.breaker);
  }

  // =========================================================================
  // CASE 3: SSN_GFL_Split
  // =========================================================================
  TimingResult runSSNGFLSplit(const String &simName,
                              const SystemTopology &systemPF) const {

    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Real>::make("nGrid", PhaseType::ABC);
    auto nSeries = SimNode<Real>::make("nSeries", PhaseType::ABC);
    auto nPcc = SimNode<Real>::make("nPcc", PhaseType::ABC);

    auto breakerLoad = createBreakerLoad(nPcc);

    auto slack = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::off);

    auto gridResistor =
        EMT::Ph3::Resistor::make("GridResistor", Logger::Level::off);
    gridResistor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridResistance));

    auto gridInductor =
        EMT::Ph3::Inductor::make("GridInductor", Logger::Level::off);
    gridInductor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridInductance));

    auto inverter =
        EMT::Ph3::SSN_GFL_Split::make("INV_GFL", Logger::Level::off);

    setStateSpaceGFLParameters(inverter);

    inverter->connect({EMT::SimNode::GND, nPcc});
    gridResistor->connect({nPcc, nSeries});
    gridInductor->connect({nSeries, nGrid});
    slack->connect({nGrid});

    auto system = SystemTopology(
        mSystemFrequency,
        SystemNodeList{nGrid, nSeries, nPcc, breakerLoad.node},
        SystemComponentList{slack, gridResistor, gridInductor, inverter,
                            breakerLoad.breaker, breakerLoad.resistor});

    system.initWithPowerflow(systemPF, Domain::EMT);

    const Real loadResistance =
        calculateLoadResistance(nPcc->initialSingleVoltage());

    breakerLoad.resistor->setParameters(loadResistance);

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_load", breakerLoad.node->attribute("v"));

    logger->logAttribute("i_breaker", breakerLoad.breaker->attribute("i_intf"));

    logger->logAttribute("i_load", breakerLoad.resistor->attribute("i_intf"));

    addStateSpaceCommonSignals(logger, nGrid, nSeries, nPcc, inverter);

    return runTimedSimulation("SSN_GFL_Split", simName, system, logger, false,
                              breakerLoad.breaker);
  }

  // =========================================================================
  // CASE 4: AvVoltSourceInverterStateSpace
  // =========================================================================
  TimingResult runAvVoltStateSpace(const String &simName,
                                   const SystemTopology &systemPF) const {

    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Real>::make("nGrid", PhaseType::ABC);
    auto nSeries = SimNode<Real>::make("nSeries", PhaseType::ABC);
    auto nPcc = SimNode<Real>::make("nPcc", PhaseType::ABC);

    auto breakerLoad = createBreakerLoad(nPcc);

    auto slack = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::off);

    auto gridResistor =
        EMT::Ph3::Resistor::make("GridResistor", Logger::Level::off);
    gridResistor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridResistance));

    auto gridInductor =
        EMT::Ph3::Inductor::make("GridInductor", Logger::Level::off);
    gridInductor->setParameters(
        Math::singlePhaseParameterToThreePhase(mGridInductance));

    auto inverter = EMT::Ph3::AvVoltSourceInverterStateSpace::make(
        "INV_GFL", Logger::Level::off);

    setStateSpaceGFLParameters(inverter);

    inverter->connect({EMT::SimNode::GND, nPcc});
    gridResistor->connect({nPcc, nSeries});
    gridInductor->connect({nSeries, nGrid});
    slack->connect({nGrid});

    auto system = SystemTopology(
        mSystemFrequency,
        SystemNodeList{nGrid, nSeries, nPcc, breakerLoad.node},
        SystemComponentList{slack, gridResistor, gridInductor, inverter,
                            breakerLoad.breaker, breakerLoad.resistor});

    system.initWithPowerflow(systemPF, Domain::EMT);

    const Real loadResistance =
        calculateLoadResistance(nPcc->initialSingleVoltage());

    breakerLoad.resistor->setParameters(loadResistance);

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_load", breakerLoad.node->attribute("v"));

    logger->logAttribute("i_breaker", breakerLoad.breaker->attribute("i_intf"));

    logger->logAttribute("i_load", breakerLoad.resistor->attribute("i_intf"));

    addStateSpaceCommonSignals(logger, nGrid, nSeries, nPcc, inverter);

    return runTimedSimulation("AvVoltSourceInverterStateSpace", simName, system,
                              logger, true, breakerLoad.breaker);
  }

  // =========================================================================
  // COMMON SIMULATION / LOGGING
  // =========================================================================
  TimingResult runTimedSimulation(
      const String &label, const String &simName, SystemTopology &system,
      const DataLogger::Ptr &logger, Bool recomputeSystemMatrix,
      const std::shared_ptr<EMT::Ph3::SeriesSwitch> &loadBreaker) const {

    Logger::setLogDir("logs/" + simName);

    Simulation sim(simName, Logger::Level::off);
    sim.setSystem(system);
    sim.addLogger(logger);
    sim.setDomain(Domain::EMT);
    sim.setSolverType(Solver::Type::MNA);
    sim.doSystemMatrixRecomputation(recomputeSystemMatrix);
    sim.doInitFromNodesAndTerminals(true);
    sim.setTimeStep(mTimeStepEMT);
    sim.setFinalTime(mFinalTimeEMT);

    // Close the physical load breaker at exactly t = 5 s.
    sim.addEvent(SwitchEvent::make(mLoadStepTime, loadBreaker, true));

    std::cout << "\nRunning " << label << "  [matrix recomputation = "
              << (recomputeSystemMatrix ? "ON" : "OFF") << ", breaker closes @ "
              << mLoadStepTime << " s"
              << "] ...\n";

    const auto start = std::chrono::steady_clock::now();

    sim.run();

    const auto stop = std::chrono::steady_clock::now();

    const double seconds = std::chrono::duration<double>(stop - start).count();

    std::cout << label << " runtime: " << std::fixed << std::setprecision(6)
              << seconds << " s\n";

    return {label, simName, seconds, recomputeSystemMatrix};
  }

  template <typename GFLPtr>
  void addStateSpaceCommonSignals(const DataLogger::Ptr &logger,
                                  const SimNode<Real>::Ptr &nGrid,
                                  const SimNode<Real>::Ptr &nSeries,
                                  const SimNode<Real>::Ptr &nPcc,
                                  const GFLPtr &inverter) const {

    logger->logAttribute("v_grid", nGrid->attribute("v"));
    logger->logAttribute("v_series", nSeries->attribute("v"));
    logger->logAttribute("v_pcc", nPcc->attribute("v"));

    logger->logAttribute("i_inv", inverter->attribute("i_intf"));

    logger->logAttribute("vc_d", inverter->attribute("vc_d"));
    logger->logAttribute("vc_q", inverter->attribute("vc_q"));

    logger->logAttribute("igrid_d", inverter->attribute("irc_d"));
    logger->logAttribute("igrid_q", inverter->attribute("irc_q"));

    logger->logAttribute("omega_pll", inverter->attribute("omega_pll"));
  }

  void addNewGFLCommonSignals(
      const DataLogger::Ptr &logger, const SimNode<Real>::Ptr &nGrid,
      const SimNode<Real>::Ptr &nSeries, const SimNode<Real>::Ptr &nPcc,
      const std::shared_ptr<EMT::Ph3::GFL> &inverter) const {

    logger->logAttribute("v_grid", nGrid->attribute("v"));
    logger->logAttribute("v_series", nSeries->attribute("v"));
    logger->logAttribute("v_pcc", nPcc->attribute("v"));

    logger->logAttribute("i_inv", inverter->attribute("i_intf"));

    logger->logAttribute("vc_d", inverter->mVcd);
    logger->logAttribute("vc_q", inverter->mVcq);
    logger->logAttribute("igrid_d", inverter->mIgridD);
    logger->logAttribute("igrid_q", inverter->mIgridQ);
    logger->logAttribute("omega_pll", inverter->mOmegaPLL);
  }

  template <typename GFLPtr>
  void setStateSpaceGFLParameters(const GFLPtr &inverter) const {

    inverter->setParameters(mLf, mCf, mRf, mRc, mSystemOmega, mKpPLL, mKiPLL,
                            mOmegaCutoff, mPrefFilter, mQrefFilter,
                            mKpPowerCtrl, mKiPowerCtrl, mKpCurrCtrl,
                            mKiCurrCtrl);
  }

  // =========================================================================
  // FILTER-SIDE -> PCC-SIDE POWER CONVERSION
  // =========================================================================
  std::pair<Real, Real>
  pccPowerFromFilterPowerReference(Real pFilterRef, Real qFilterRef) const {

    const Real vPccPeakPhase = RMS3PH_TO_PEAK1PH * mGridVoltageRMSLineToLine;

    if (std::abs(mRc) < 1e-12 || vPccPeakPhase < 1e-9)
      return {pFilterRef, qFilterRef};

    const Real qPccRef = qFilterRef;

    const Real a = mRc / (1.5 * vPccPeakPhase * vPccPeakPhase);

    const Real discriminant =
        1.0 + 4.0 * a * (pFilterRef - a * qPccRef * qPccRef);

    if (discriminant < 0.0)
      throw std::runtime_error(
          "No feasible PCC power for requested filter-side P/Q.");

    const Real sqrtDisc = std::sqrt(discriminant);

    const Real p1 = (-1.0 + sqrtDisc) / (2.0 * a);

    const Real p2 = (-1.0 - sqrtDisc) / (2.0 * a);

    const Real pPccRef =
        std::abs(p1 - pFilterRef) < std::abs(p2 - pFilterRef) ? p1 : p2;

    return {pPccRef, qPccRef};
  }

  // =========================================================================
  // OUTPUT
  // =========================================================================
  void printConfiguration(Real pPccRef, Real qPccRef) const {

    std::cout
        << "\n"
        << "============================================================\n"
        << " FOUR-MODEL GFL BREAKER LOAD-STEP BENCHMARK\n"
        << "============================================================\n"
        << "Grid      : GFL -- R -- L -- NetworkInjection\n"
        << "dt        : " << mTimeStepEMT << " s\n"
        << "T_end     : " << mFinalTimeEMT << " s\n"
        << "steps     : "
        << static_cast<long long>(std::llround(mFinalTimeEMT / mTimeStepEMT))
        << "\n"
        << "V_grid    : " << mGridVoltageRMSLineToLine << " V LL RMS\n"
        << "R_grid    : " << mGridResistance << " Ohm\n"
        << "L_grid    : " << mGridInductance << " H\n"
        << "\n"
        << "Breaker load step:\n"
        << "  t_close : " << mLoadStepTime << " s\n"
        << "  P_load  : " << mLoadStepP << " W nominal\n"
        << "  Q_load  : 0 var (pure R)\n"
        << "  R_open  : " << mBreakerOpenResistance << " Ohm\n"
        << "  R_closed: " << mBreakerClosedResistance << " Ohm\n"
        << "  logging : use i_breaker/i_load to verify closure\n"
        << "\n"
        << "Rc-based SSN/state-space capacitor-side reference:\n"
        << "  P_vc    : " << mPrefFilter << " W\n"
        << "  Q_vc    : " << mQrefFilter << " var\n"
        << "\n"
        << "Equivalent PCC operating point / new GFL reference:\n"
        << "  P_pcc   : " << pPccRef << " W\n"
        << "  Q_pcc   : " << qPccRef << " var\n"
        << "============================================================\n";
  }

  void printTimingSummary(const std::vector<TimingResult> &timings) const {

    if (timings.empty())
      return;

    const long long steps =
        static_cast<long long>(std::llround(mFinalTimeEMT / mTimeStepEMT));

    const double referenceTime = timings.front().wallSeconds;

    double fastestTime = timings.front().wallSeconds;

    for (const auto &result : timings)
      fastestTime = std::min(fastestTime, result.wallSeconds);

    std::cout << "\n\n"
              << "============================================================="
                 "=============\n"
              << " TIMING SUMMARY\n"
              << "============================================================="
                 "=============\n";

    std::cout << std::left << std::setw(34) << "Model" << std::right
              << std::setw(13) << "time [s]" << std::setw(16) << "us/step"
              << std::setw(15) << "vs GFL" << std::setw(12) << "matrix"
              << "\n";

    std::cout << "-------------------------------------------------------------"
                 "-------------\n";

    for (const auto &result : timings) {
      const double usPerStep =
          1e6 * result.wallSeconds / static_cast<double>(steps);

      const double speedupVsReference =
          result.wallSeconds > 0.0 ? referenceTime / result.wallSeconds : 0.0;

      std::cout << std::left << std::setw(34) << result.label << std::right
                << std::fixed << std::setprecision(6) << std::setw(13)
                << result.wallSeconds << std::setw(16) << usPerStep
                << std::setw(14) << speedupVsReference << "x" << std::setw(12)
                << (result.recomputeSystemMatrix ? "recompute" : "fixed")
                << "\n";
    }

    std::cout << "-------------------------------------------------------------"
                 "-------------\n"
              << "Fastest wall time: " << fastestTime << " s\n"
              << "============================================================="
                 "=============\n"
              << "\nCSV files for direct comparison:\n";

    for (const auto &result : timings)
      std::cout << "  " << result.simulationName << ".csv\n";

    std::cout << "\nAll four CSVs contain the same benchmark signal names:\n"
              << "  v_grid, v_series, v_pcc, i_inv,\n"
              << "  v_load, i_breaker, i_load,\n"
              << "  vc_d, vc_q, igrid_d, igrid_q, omega_pll\n";
  }

private:
  Real mTimeStepEMT;
  Real mFinalTimeEMT;

  Real mSystemFrequency;
  Real mSystemOmega;
  Real mGridVoltageRMSLineToLine;

  Real mGridResistance;
  Real mGridInductance;

  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;

  Real mKpPLL;
  Real mKiPLL;

  Real mOmegaCutoff;

  // Filter-capacitor-side references used by the three SSN/state-space models.
  Real mPrefFilter;
  Real mQrefFilter;

  Real mKpPowerCtrl;
  Real mKiPowerCtrl;
  Real mKpCurrCtrl;
  Real mKiCurrCtrl;

  Real mLoadStepTime;
  Real mLoadStepP;
  Real mBreakerOpenResistance;
  Real mBreakerClosedResistance;
};

int main(int argc, char *argv[]) {
  Example_GFL_Four_Model_BreakerLoadStep example;
  example.run();
  return 0;
}
