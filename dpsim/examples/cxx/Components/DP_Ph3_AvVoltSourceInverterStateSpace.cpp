// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

class Example_AvVoltSourceInverterStateSpace {
public:
  Example_AvVoltSourceInverterStateSpace()
      : mTimeStepDP(100e-6), mFinalTimeDP(1.0), mFaultTime(0.1),
        mSystemFrequency(50.0), mSystemOmega(2.0 * PI * mSystemFrequency),
        mGridVoltageRMSLineToLine(400.0), mLineResistance(0.3),
        mLineInductance(0.1e-3), mLineCapacitance(1e-6), mLineConductance(1e-6),
        mFaultResistance(1.0), mSwitchOpenResistance(1e12), mLf(2e-3),
        mCf(10e-6), mRf(0.2), mRc(0.2), mKpPLL(0.25), mKiPLL(0.2),
        mOmegaCutoff(mSystemOmega), mPref(10000.0), mQref(5000.0),
        mKpPowerCtrl(0.05), mKiPowerCtrl(0.2), mKpCurrCtrl(0.25),
        mKiCurrCtrl(1.0) {}

  void run() const {
    const String simNameBase = "DP_Ph3_AvVoltSourceInverterStateSpace";

    const auto systemPF = runPowerFlow(simNameBase + "_PF");
    runDPSimulation(simNameBase + "_DP", systemPF);
  }

private:
  SystemTopology runPowerFlow(const String &simName) const {
    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Complex>::make("nGrid", PhaseType::Single);
    auto nPcc = SimNode<Complex>::make("nPcc", PhaseType::Single);

    auto slack = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::info);
    slack->setParameters(mGridVoltageRMSLineToLine);
    slack->setBaseVoltage(mGridVoltageRMSLineToLine);
    slack->modifyPowerFlowBusType(PowerflowBusType::VD);

    auto line = SP::Ph1::PiLine::make("Line", Logger::Level::info);
    line->setParameters(mLineResistance, mLineInductance, mLineCapacitance,
                        mLineConductance);
    line->setBaseVoltage(mGridVoltageRMSLineToLine);

    // The inverter controller regulates filter-side power, i.e. the power at
    // Vc through Rc. The power-flow injection is therefore corrected to the
    // corresponding PCC-side power. The fault is a timed event and is not part
    // of the balanced power-flow initialization.
    const auto [pPccRef, qPccRef] =
        pccPowerFromFilterPowerReference(mPref, mQref);

    // Negative load = injected P/Q at PCC.
    auto inverterInjection =
        SP::Ph1::Load::make("INV_SSN_PLL", Logger::Level::info);
    inverterInjection->setParameters(-pPccRef, -qPccRef,
                                     mGridVoltageRMSLineToLine);
    inverterInjection->modifyPowerFlowBusType(PowerflowBusType::PQ);

    slack->connect({nGrid});
    line->connect({nGrid, nPcc});
    inverterInjection->connect({nPcc});

    auto system =
        SystemTopology(mSystemFrequency, SystemNodeList{nGrid, nPcc},
                       SystemComponentList{slack, line, inverterInjection});

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_grid_pf", nGrid->attribute("v"));
    logger->logAttribute("v_pcc_pf", nPcc->attribute("v"));

    const Real timeStepPF = mFinalTimeDP;
    const Real finalTimePF = mFinalTimeDP + timeStepPF;

    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(system);
    sim.setTimeStep(timeStepPF);
    sim.setFinalTime(finalTimePF);
    sim.setDomain(Domain::SP);
    sim.setSolverType(Solver::Type::NRP);
    sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
    sim.doInitFromNodesAndTerminals(false);
    sim.addLogger(logger);
    sim.run();

    return system;
  }

  void runDPSimulation(const String &simName,
                       const SystemTopology &systemPF) const {
    Logger::setLogDir("logs/" + simName);

    auto nGrid = SimNode<Complex>::make("nGrid", PhaseType::ABC);
    auto nPcc = SimNode<Complex>::make("nPcc", PhaseType::ABC);

    auto slack = DP::Ph3::NetworkInjection::make("Slack", Logger::Level::info);

    auto line = DP::Ph3::PiLine::make("Line", Logger::Level::info);
    line->setParameters(
        Math::singlePhaseParameterToThreePhase(mLineResistance),
        Math::singlePhaseParameterToThreePhase(mLineInductance),
        Math::singlePhaseParameterToThreePhase(mLineCapacitance),
        Math::singlePhaseParameterToThreePhase(mLineConductance));

    auto inverter = DP::Ph3::AvVoltSourceInverterStateSpace::make(
        "INV_SSN_PLL", Logger::Level::info);
    inverter->setParameters(mLf, mCf, mRf, mRc, mSystemOmega, mKpPLL, mKiPLL,
                            mOmegaCutoff, mPref, mQref, mKpPowerCtrl,
                            mKiPowerCtrl, mKpCurrCtrl, mKiCurrCtrl);

    // Single-line-to-ground fault: ganged switch, closed matrix conducts only
    // phase A through a moderate resistance; closes at mFaultTime.
    auto fault = DP::Ph3::Switch::make("Fault", Logger::Level::info);
    fault->setParameters(openResistance(), closedResistance(), false);

    slack->connect({nGrid});
    line->connect({nGrid, nPcc});

    // terminal 0 = GND, terminal 1 = nPcc
    inverter->connect({DP::SimNode::GND, nPcc});

    // Fault branch shunts the PCC to ground, closed by the timed event.
    fault->connect({nPcc, DP::SimNode::GND});

    auto system =
        SystemTopology(mSystemFrequency, SystemNodeList{nGrid, nPcc},
                       SystemComponentList{slack, line, inverter, fault});

    system.initWithPowerflow(systemPF, Domain::DP);

    auto logger = DataLogger::make(simName);
    logger->logAttribute("v_pcc", nPcc->attribute("v"));
    logger->logAttribute("i_inv", inverter->attribute("i_intf"));
    logger->logAttribute("i_fault", fault->attribute("i_intf"));
    logger->logAttribute("vc_d", inverter->attribute("vc_d"));
    logger->logAttribute("vc_q", inverter->attribute("vc_q"));
    logger->logAttribute("irc_n_d", inverter->attribute("irc_n_d"));
    logger->logAttribute("irc_n_q", inverter->attribute("irc_n_q"));
    logger->logAttribute("p_inst", inverter->attribute("p_inst"));
    logger->logAttribute("q_inst", inverter->attribute("q_inst"));
    logger->logAttribute("omega_pll", inverter->attribute("omega_pll"));

    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(system);
    sim.addLogger(logger);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.doSystemMatrixRecomputation(true);
    sim.doInitFromNodesAndTerminals(true);

    sim.addEvent(SwitchEvent3Ph::make(mFaultTime, fault, true));

    sim.setTimeStep(mTimeStepDP);
    sim.setFinalTime(mFinalTimeDP);
    sim.run();
  }

  Matrix openResistance() const {
    return Math::singlePhaseParameterToThreePhase(mSwitchOpenResistance);
  }

  // Phase A faults through the fault resistance; B and C stay open.
  Matrix closedResistance() const {
    Matrix r = Math::singlePhaseParameterToThreePhase(mSwitchOpenResistance);
    r(0, 0) = mFaultResistance;
    return r;
  }

  std::pair<Real, Real>
  pccPowerFromFilterPowerReference(Real pFilterRef, Real qFilterRef) const {
    const Real vPccPeakPhase = RMS3PH_TO_PEAK1PH * mGridVoltageRMSLineToLine;

    if (std::abs(mRc) < 1e-12 || vPccPeakPhase < 1e-9)
      return {pFilterRef, qFilterRef};

    // With peak phase phasors:
    //
    // P_filter = P_pcc + Rc / (1.5 * |U|^2) * (P_pcc^2 + Q_pcc^2)
    // Q_filter = Q_pcc
    const Real qPccRef = qFilterRef;
    const Real a = mRc / (1.5 * vPccPeakPhase * vPccPeakPhase);

    const Real discriminant =
        1.0 + 4.0 * a * (pFilterRef - a * qPccRef * qPccRef);

    if (discriminant < 0.0) {
      throw std::runtime_error(
          "No feasible PCC power found for the requested filter-side power "
          "reference, Rc, and PCC voltage estimate.");
    }

    const Real sqrtDisc = std::sqrt(discriminant);
    const Real p1 = (-1.0 + sqrtDisc) / (2.0 * a);
    const Real p2 = (-1.0 - sqrtDisc) / (2.0 * a);

    // Select the root close to pFilterRef, which is the physically relevant
    // branch for small Rc.
    const Real pPccRef =
        std::abs(p1 - pFilterRef) < std::abs(p2 - pFilterRef) ? p1 : p2;

    return {pPccRef, qPccRef};
  }

private:
  Real mTimeStepDP;
  Real mFinalTimeDP;
  Real mFaultTime;

  Real mSystemFrequency;
  Real mSystemOmega;
  Real mGridVoltageRMSLineToLine;

  Real mLineResistance;
  Real mLineInductance;
  Real mLineCapacitance;
  Real mLineConductance;

  Real mFaultResistance;
  Real mSwitchOpenResistance;

  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;

  Real mKpPLL;
  Real mKiPLL;

  Real mOmegaCutoff;
  Real mPref;
  Real mQref;
  Real mKpPowerCtrl;
  Real mKiPowerCtrl;
  Real mKpCurrCtrl;
  Real mKiCurrCtrl;
};

int main(int argc, char *argv[]) {
  Example_AvVoltSourceInverterStateSpace example;
  example.run();
  return 0;
}
