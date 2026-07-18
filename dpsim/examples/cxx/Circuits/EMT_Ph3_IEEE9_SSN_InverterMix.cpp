// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "../Examples.h"
#include "../GeneratorFactory.h"

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {
// GFL avg-VSI filter and gains: the 400 V/10 kVA reference design rebased to
// the BUS3 base (13.8 kV, 100 MVA, 60 Hz), preserving the per-unit values.
struct GflParams {
  Real lf = 1.98375e-4; // H
  Real cf = 7.00133e-5; // F
  Real rf = 2.38050e-2; // Ohm
  Real rc = 2.38050e-2; // Ohm
  Real kpPLL = 7.246377e-3;
  Real kiPLL = 5.797101e-3;
  Real kpPowerCtrl = 1.449275e-3;
  Real kiPowerCtrl = 5.797101e-3;
  Real kpCurrCtrl = 2.975625e-2;
  Real kiCurrCtrl = 1.190250e-1;
};

// GFM SSN filter/VSG rebased from the PR #570 reference (220 V/12 kVA/50 Hz) to
// the BUS2 base (18 kV, 100 MVA, 60 Hz), with grid-connected control tuning.
struct GfmParams {
  Real lf = 6.694215e-4; // H
  Real cf = 6.224280e-5; // F
  Real rf = 1.338843e-2; // Ohm
  Real rc = 1.338843e-2; // Ohm

  Real virtualInertia = 1157.407407;
  Real dampingCoefficient = 3.0e6; // raised for stiff-grid stability

  // Superseded by the proportional Q-V droop below.
  Real voltageDroopGain = 1.0 / 15.0;
  Real reactiveIntegralGain = 1.700559e-3;

  Real kpVoltage = 5.8e-3; // lowered for stiff-grid stability
  Real kiVoltage = 2.7e-1;
  Real kpCurrent = 1.126636e0;
  Real kiCurrent = 2.253273e1;

  Real activeDampingGain = 0.0;
  Real powerFilterCutoff = 100.0;
  Real delayBandwidth = 20e3 / 1.5;

  Real reactivePowerDroop = 1.0e-5; // proportional Q-V droop
  Real reactiveDroopCutoff = 100.0;
};

// Filter-side power reference to PCC-side injection, correcting the rc loss.
std::pair<Real, Real> pccPowerFromFilterPowerReference(Real pFilterRef,
                                                       Real qFilterRef, Real rc,
                                                       Real vGridRmsLL) {
  const Real vPccPeakPhase = RMS3PH_TO_PEAK1PH * vGridRmsLL;
  if (std::abs(rc) < 1e-12 || vPccPeakPhase < 1e-9)
    return {pFilterRef, qFilterRef};

  const Real qPccRef = qFilterRef;
  const Real a = rc / (1.5 * vPccPeakPhase * vPccPeakPhase);
  const Real discriminant =
      1.0 + 4.0 * a * (pFilterRef - a * qPccRef * qPccRef);
  if (discriminant < 0.0)
    throw std::runtime_error("No feasible PCC power for the GFL filter-side "
                             "power reference, rc, and PCC voltage estimate.");

  const Real sqrtDisc = std::sqrt(discriminant);
  const Real p1 = (-1.0 + sqrtDisc) / (2.0 * a);
  const Real p2 = (-1.0 - sqrtDisc) / (2.0 * a);
  const Real pPccRef =
      std::abs(p1 - pFilterRef) < std::abs(p2 - pFilterRef) ? p1 : p2;
  return {pPccRef, qPccRef};
}
} // namespace

SystemTopology buildTopology(CommandLineArgs &args,
                             std::shared_ptr<DataLoggerInterface> logger) {

  String simName = args.name;

  CPS::CIM::Examples::Grids::IEEE9::ScenarioConfig ieee9(args.sysFreq);

  // POWER FLOW FOR INITIALIZATION
  CPS::Logger::get(args.name)->info("Creating power flow initialization.");

  String simNamePF = simName + "_PF";
  CPS::Logger::setLogDir("logs/" + simNamePF);

  // Nodes
  auto n1PF = SimNode<Complex>::make("BUS1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("BUS2", PhaseType::Single);
  auto n3PF = SimNode<Complex>::make("BUS3", PhaseType::Single);
  auto n4PF = SimNode<Complex>::make("BUS4", PhaseType::Single);
  auto n5PF = SimNode<Complex>::make("BUS5", PhaseType::Single);
  auto n6PF = SimNode<Complex>::make("BUS6", PhaseType::Single);
  auto n7PF = SimNode<Complex>::make("BUS7", PhaseType::Single);
  auto n8PF = SimNode<Complex>::make("BUS8", PhaseType::Single);
  auto n9PF = SimNode<Complex>::make("BUS9", PhaseType::Single);

  auto gen1PF = SP::Ph1::SynchronGenerator::make(ieee9.gen1.Name,
                                                 CPS::Logger::Level::off);
  gen1PF->setParameters(ieee9.gen1.RatedPower, ieee9.gen1.RatedVoltage,
                        ieee9.gen1.InitialPower, ieee9.gen1.InitialVoltage,
                        ieee9.gen1.BusType);
  gen1PF->setBaseVoltage(ieee9.gen1.RatedVoltage);

  auto gen2PF = SP::Ph1::SynchronGenerator::make(ieee9.gen2.Name,
                                                 CPS::Logger::Level::off);
  gen2PF->setParameters(ieee9.gen2.RatedPower, ieee9.gen2.RatedVoltage,
                        ieee9.gen2.InitialPower, ieee9.gen2.InitialVoltage,
                        ieee9.gen2.BusType);
  gen2PF->setBaseVoltage(ieee9.gen2.RatedVoltage);

  // gen3's PF image: a negative PQ load injecting the PCC-side power (the
  // rc-corrected filter reference).
  const GflParams gfl;
  const auto [gfl3PPcc, gfl3QPcc] = pccPowerFromFilterPowerReference(
      ieee9.gen3.InitialPower, ieee9.gen3.InitialPowerReactive, gfl.rc,
      ieee9.gen3.RatedVoltage);
  auto gfl3PF = SP::Ph1::Load::make(ieee9.gen3.Name, CPS::Logger::Level::off);
  gfl3PF->setParameters(-gfl3PPcc, -gfl3QPcc, ieee9.gen3.RatedVoltage);
  gfl3PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

  // Loads
  auto load5PF = SP::Ph1::Load::make(ieee9.load5.Name, CPS::Logger::Level::off);
  load5PF->setParameters(ieee9.load5.RealPower, ieee9.load5.ReactivePower,
                         ieee9.load5.BaseVoltage);
  load5PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

  auto load6PF = SP::Ph1::Load::make(ieee9.load6.Name, CPS::Logger::Level::off);
  load6PF->setParameters(ieee9.load6.RealPower, ieee9.load6.ReactivePower,
                         ieee9.load6.BaseVoltage);
  load6PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

  auto load8PF = SP::Ph1::Load::make(ieee9.load8.Name, CPS::Logger::Level::off);
  load8PF->setParameters(ieee9.load8.RealPower, ieee9.load8.ReactivePower,
                         ieee9.load8.BaseVoltage);
  load8PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

  // Transmission Lines

  auto line54PF =
      SP::Ph1::PiLine::make(ieee9.line54.Name, CPS::Logger::Level::off);
  line54PF->setParameters(ieee9.line54.Resistance, ieee9.line54.Inductance,
                          ieee9.line54.Capacitance, ieee9.line54.Conductance);
  line54PF->setBaseVoltage(ieee9.line54.BaseVoltage);

  auto line64PF =
      SP::Ph1::PiLine::make(ieee9.line64.Name, CPS::Logger::Level::off);
  line64PF->setParameters(ieee9.line64.Resistance, ieee9.line64.Inductance,
                          ieee9.line64.Capacitance, ieee9.line64.Conductance);
  line64PF->setBaseVoltage(ieee9.line64.BaseVoltage);

  auto line75PF =
      SP::Ph1::PiLine::make(ieee9.line75.Name, CPS::Logger::Level::off);
  line75PF->setParameters(ieee9.line75.Resistance, ieee9.line75.Inductance,
                          ieee9.line75.Capacitance, ieee9.line75.Conductance);
  line75PF->setBaseVoltage(ieee9.line75.BaseVoltage);

  auto line96PF =
      SP::Ph1::PiLine::make(ieee9.line96.Name, CPS::Logger::Level::off);
  line96PF->setParameters(ieee9.line96.Resistance, ieee9.line96.Inductance,
                          ieee9.line96.Capacitance, ieee9.line96.Conductance);
  line96PF->setBaseVoltage(ieee9.line96.BaseVoltage);

  auto line78PF =
      SP::Ph1::PiLine::make(ieee9.line78.Name, CPS::Logger::Level::off);
  line78PF->setParameters(ieee9.line78.Resistance, ieee9.line78.Inductance,
                          ieee9.line78.Capacitance, ieee9.line78.Conductance);
  line78PF->setBaseVoltage(ieee9.line78.BaseVoltage);

  auto line89PF =
      SP::Ph1::PiLine::make(ieee9.line89.Name, CPS::Logger::Level::off);
  line89PF->setParameters(ieee9.line89.Resistance, ieee9.line89.Inductance,
                          ieee9.line89.Capacitance, ieee9.line89.Conductance);
  line89PF->setBaseVoltage(ieee9.line89.BaseVoltage);

  // Transformers

  auto transf14PF =
      SP::Ph1::Transformer::make(ieee9.transf14.Name, CPS::Logger::Level::off);
  transf14PF->setParameters(
      ieee9.transf14.VoltageLVSide, ieee9.transf14.VoltageHVSide,
      ieee9.transf14.Ratio, 0.0, // No phase shift (ratioPhase = 0.0)
      ieee9.transf14.Resistance, ieee9.transf14.Inductance);
  transf14PF->setBaseVoltage(ieee9.transf14.VoltageHVSide);

  auto transf27PF =
      SP::Ph1::Transformer::make(ieee9.transf27.Name, CPS::Logger::Level::off);
  transf27PF->setParameters(ieee9.transf27.VoltageLVSide,
                            ieee9.transf27.VoltageHVSide, ieee9.transf27.Ratio,
                            0.0, ieee9.transf27.Resistance,
                            ieee9.transf27.Inductance);
  transf27PF->setBaseVoltage(ieee9.transf27.VoltageHVSide);

  auto transf39PF =
      SP::Ph1::Transformer::make(ieee9.transf39.Name, CPS::Logger::Level::off);
  transf39PF->setParameters(ieee9.transf39.VoltageLVSide,
                            ieee9.transf39.VoltageHVSide, ieee9.transf39.Ratio,
                            0.0, ieee9.transf39.Resistance,
                            ieee9.transf39.Inductance);
  transf39PF->setBaseVoltage(ieee9.transf39.VoltageHVSide);

  // Connect components
  gen1PF->connect({n1PF});
  gen2PF->connect({n2PF});
  gfl3PF->connect({n3PF});

  load5PF->connect({n5PF});
  load6PF->connect({n6PF});
  load8PF->connect({n8PF});

  line54PF->connect({n5PF, n4PF});
  line64PF->connect({n6PF, n4PF});
  line75PF->connect({n7PF, n5PF});
  line96PF->connect({n9PF, n6PF});
  line78PF->connect({n7PF, n8PF});
  line89PF->connect({n8PF, n9PF});

  transf14PF->connect({n1PF, n4PF});
  transf27PF->connect({n2PF, n7PF});
  transf39PF->connect({n3PF, n9PF});

  // Create system topology
  auto systemPF = SystemTopology(
      ieee9.nomFreq,
      SystemNodeList{n1PF, n2PF, n3PF, n4PF, n5PF, n6PF, n7PF, n8PF, n9PF},
      SystemComponentList{gen1PF, gen2PF, gfl3PF, load5PF, load6PF, load8PF,
                          line54PF, line64PF, line75PF, line96PF, line78PF,
                          line89PF, transf14PF, transf27PF, transf39PF});

  // Logger
  auto loggerPF = DataLogger::make(simNamePF, CPS::Logger::Level::off);
  // Log node voltages
  loggerPF->logAttribute("v_bus1", n1PF->attribute("v"));
  loggerPF->logAttribute("v_bus2", n2PF->attribute("v"));
  loggerPF->logAttribute("v_bus3", n3PF->attribute("v"));
  loggerPF->logAttribute("v_bus4", n4PF->attribute("v"));
  loggerPF->logAttribute("v_bus5", n5PF->attribute("v"));
  loggerPF->logAttribute("v_bus6", n6PF->attribute("v"));
  loggerPF->logAttribute("v_bus7", n7PF->attribute("v"));
  loggerPF->logAttribute("v_bus8", n8PF->attribute("v"));
  loggerPF->logAttribute("v_bus9", n9PF->attribute("v"));
  // Log node powers
  loggerPF->logAttribute("s_bus1", n1PF->attribute("s"));
  loggerPF->logAttribute("s_bus2", n2PF->attribute("s"));
  loggerPF->logAttribute("s_bus3", n3PF->attribute("s"));
  loggerPF->logAttribute("s_bus4", n4PF->attribute("s"));
  loggerPF->logAttribute("s_bus5", n5PF->attribute("s"));
  loggerPF->logAttribute("s_bus6", n6PF->attribute("s"));
  loggerPF->logAttribute("s_bus7", n7PF->attribute("s"));
  loggerPF->logAttribute("s_bus8", n8PF->attribute("s"));
  loggerPF->logAttribute("s_bus9", n9PF->attribute("s"));

  // Run power flow simulation
  Simulation simPF(simNamePF, CPS::Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(args.timeStep);
  simPF.setFinalTime(1 * args.timeStep);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Simulation);
  simPF.addLogger(loggerPF);
  simPF.run();
  CPS::Logger::get(args.name)->info("Power flow simulation finished.");

  // Seed voltages for the step-2 inverters: converged n2 (GFM PCC) and
  // n3 (GFL PCC) node voltages, magnitude [kV] and angle [deg]. Dedicated
  // console-enabled logger so the seeds survive the setLogDir churn.
  Complex v2PF = n2PF->singleVoltage();
  Complex v3PF = n3PF->singleVoltage();
  auto seedLog = CPS::Logger::get(simNamePF + "_seed", CPS::Logger::Level::info,
                                  CPS::Logger::Level::info);
  seedLog->info(
      "PF seed n2 (BUS2): |V| = {:.4f} kV ({:.4f} pu), angle = {:.4f} deg",
      std::abs(v2PF) / 1e3, std::abs(v2PF) / ieee9.gen2.RatedVoltage,
      std::arg(v2PF) * 180.0 / PI);
  seedLog->info(
      "PF seed n3 (BUS3): |V| = {:.4f} kV ({:.4f} pu), angle = {:.4f} deg",
      std::abs(v3PF) / 1e3, std::abs(v3PF) / ieee9.gen3.RatedVoltage,
      std::arg(v3PF) * 180.0 / PI);

  // DYNAMIC SIMULATION - EMT
  CPS::Logger::get(args.name)->info("Dynamic simulation initialization.");
  String simNameEMT = simName + "_EMT";
  CPS::Logger::setLogDir("logs/" + simNameEMT);

  // Nodes
  auto n1EMT = SimNode<Real>::make("BUS1", PhaseType::ABC);
  auto n2EMT = SimNode<Real>::make("BUS2", PhaseType::ABC);
  auto n3EMT = SimNode<Real>::make("BUS3", PhaseType::ABC);
  auto n4EMT = SimNode<Real>::make("BUS4", PhaseType::ABC);
  auto n5EMT = SimNode<Real>::make("BUS5", PhaseType::ABC);
  auto n6EMT = SimNode<Real>::make("BUS6", PhaseType::ABC);
  auto n7EMT = SimNode<Real>::make("BUS7", PhaseType::ABC);
  auto n8EMT = SimNode<Real>::make("BUS8", PhaseType::ABC);
  auto n9EMT = SimNode<Real>::make("BUS9", PhaseType::ABC);

  // Generators
  auto gen1EMT = EMT::Ph3::SynchronGenerator4OrderVBR::make(
      ieee9.gen1.Name, CPS::Logger::Level::off);

  gen1EMT->setOperationalParametersPerUnit(
      ieee9.gen1.RatedPower,   // nomPower [VA]
      ieee9.gen1.RatedVoltage, // nomVolt [V]
      ieee9.nomFreq,           // nomFreq [Hz]
      ieee9.gen1.H, ieee9.gen1.Xd, ieee9.gen1.Xq, ieee9.gen1.Xa,
      ieee9.gen1.XdPrime, ieee9.gen1.XqPrime, ieee9.gen1.TdoPrime,
      ieee9.gen1.TqoPrime);

  auto exciter1Params = std::make_shared<Signal::ExciterDC1SimpParameters>();
  exciter1Params->Ta = ieee9.exc1.TA;
  exciter1Params->Ka = ieee9.exc1.KA;
  exciter1Params->Tef = ieee9.exc1.TE;
  exciter1Params->Kef = ieee9.exc1.KE;
  exciter1Params->Tf = ieee9.exc1.TF;
  exciter1Params->Kf = ieee9.exc1.KF;
  exciter1Params->Tr = 0.01;
  exciter1Params->MaxVa = ieee9.exc1.VRmax;
  exciter1Params->MinVa = ieee9.exc1.VRmin;
  exciter1Params->Bef = std::log(ieee9.exc1.S_EX2 / ieee9.exc1.S_EX1) /
                        (ieee9.exc1.EX2 - ieee9.exc1.EX1);
  exciter1Params->Aef =
      ieee9.exc1.S_EX1 / std::exp(exciter1Params->Bef * ieee9.exc1.EX1);
  auto exciter1 =
      Signal::ExciterDC1Simp::make("Gen1_Exciter", CPS::Logger::Level::off);
  exciter1->setParameters(exciter1Params);
  gen1EMT->addExciter(exciter1);

  // Adaptation of the governor model parameters to the dpsim implementation
  CPS::Real T4 = 1.0;
  CPS::Real T5 = 1.0;

  std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernor1 =
      Signal::TurbineGovernorType1::make("Gen1_TurbineGovernor",
                                         CPS::Logger::Level::off);

  turbineGovernor1->setParameters(ieee9.gov1.T2, T4, T5, ieee9.gov1.T3,
                                  ieee9.gov1.T1, ieee9.gov1.R, ieee9.gov1.Vmin,
                                  ieee9.gov1.Vmax, 1.0);

  gen1EMT->addGovernor(turbineGovernor1);

  const Real omegaN = 2.0 * PI * ieee9.nomFreq;

  // gen2 replaced by a grid-forming SSN inverter, keeping the GEN2 identity.
  // nominalVoltage is the peak phase target at the 1.025 pu PV setpoint.
  const Real gfmNominalVoltage = RMS3PH_TO_PEAK1PH * ieee9.gen2.InitialVoltage;
  const GfmParams gfm;
  auto gen2EMT = EMT::Ph3::SSN_GFM::make(ieee9.gen2.Name, ieee9.gen2.Name,
                                         CPS::Logger::Level::off);
  gen2EMT->setNumericalLinearizationParameters(1e-6, 1e-8);
  gen2EMT->setParameters(gfm.lf, gfm.cf, gfm.rf, gfm.rc, gfmNominalVoltage,
                         omegaN, ieee9.gen2.InitialPower,
                         ieee9.gen2.InitialPowerReactive, gfm.virtualInertia,
                         gfm.dampingCoefficient, gfm.voltageDroopGain,
                         gfm.reactiveIntegralGain, gfm.kpVoltage, gfm.kiVoltage,
                         gfm.kpCurrent, gfm.kiCurrent, gfm.activeDampingGain,
                         gfm.powerFilterCutoff, gfm.delayBandwidth);
  // Grid-connected control: no grid-current feedforward, proportional Q-V droop.
  gen2EMT->setGridCurrentFeedforward(0.0);
  gen2EMT->setReactivePowerDroop(gfm.reactivePowerDroop,
                                 gfm.reactiveDroopCutoff);

  // gen3 replaced by a grid-following averaged VSI (SSN), keeping the GEN3
  // identity so the topology wiring is unchanged.
  auto gen3EMT = EMT::Ph3::AvVoltSourceInverterStateSpace::make(
      ieee9.gen3.Name, CPS::Logger::Level::off);
  gen3EMT->setParameters(gfl.lf, gfl.cf, gfl.rf, gfl.rc, omegaN, gfl.kpPLL,
                         gfl.kiPLL, omegaN, ieee9.gen3.InitialPower,
                         ieee9.gen3.InitialPowerReactive, gfl.kpPowerCtrl,
                         gfl.kiPowerCtrl, gfl.kpCurrCtrl, gfl.kiCurrCtrl);

  // Loads
  auto load5EMT =
      EMT::Ph3::RXLoad::make(ieee9.load5.Name, CPS::Logger::Level::off);
  load5EMT->setParameters(
      Math::singlePhasePowerToThreePhase(ieee9.load5.RealPower),
      Math::singlePhasePowerToThreePhase(ieee9.load5.ReactivePower),
      ieee9.load5.BaseVoltage);

  auto load6EMT =
      EMT::Ph3::RXLoad::make(ieee9.load6.Name, CPS::Logger::Level::off);
  load6EMT->setParameters(
      Math::singlePhasePowerToThreePhase(ieee9.load6.RealPower),
      Math::singlePhasePowerToThreePhase(ieee9.load6.ReactivePower),
      ieee9.load6.BaseVoltage);

  auto load8EMT =
      EMT::Ph3::RXLoad::make(ieee9.load8.Name, CPS::Logger::Level::off);
  load8EMT->setParameters(
      Math::singlePhasePowerToThreePhase(ieee9.load8.RealPower),
      Math::singlePhasePowerToThreePhase(ieee9.load8.ReactivePower),
      ieee9.load8.BaseVoltage);

  // Lines
  auto line54EMT =
      EMT::Ph3::PiLine::make(ieee9.line54.Name, CPS::Logger::Level::off);
  line54EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(ieee9.line54.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.line54.Inductance),
      Math::singlePhaseParameterToThreePhase(ieee9.line54.Capacitance),
      Math::singlePhaseParameterToThreePhase(ieee9.line54.Conductance));

  auto line64EMT =
      EMT::Ph3::PiLine::make(ieee9.line64.Name, CPS::Logger::Level::off);
  line64EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(ieee9.line64.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.line64.Inductance),
      Math::singlePhaseParameterToThreePhase(ieee9.line64.Capacitance),
      Math::singlePhaseParameterToThreePhase(ieee9.line64.Conductance));

  auto line75EMT =
      EMT::Ph3::PiLine::make(ieee9.line75.Name, CPS::Logger::Level::off);
  line75EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(ieee9.line75.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.line75.Inductance),
      Math::singlePhaseParameterToThreePhase(ieee9.line75.Capacitance),
      Math::singlePhaseParameterToThreePhase(ieee9.line75.Conductance));

  auto line96EMT =
      EMT::Ph3::PiLine::make(ieee9.line96.Name, CPS::Logger::Level::off);
  line96EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(ieee9.line96.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.line96.Inductance),
      Math::singlePhaseParameterToThreePhase(ieee9.line96.Capacitance),
      Math::singlePhaseParameterToThreePhase(ieee9.line96.Conductance));

  auto line78EMT =
      EMT::Ph3::PiLine::make(ieee9.line78.Name, CPS::Logger::Level::off);
  line78EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(ieee9.line78.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.line78.Inductance),
      Math::singlePhaseParameterToThreePhase(ieee9.line78.Capacitance),
      Math::singlePhaseParameterToThreePhase(ieee9.line78.Conductance));

  auto line89EMT =
      EMT::Ph3::PiLine::make(ieee9.line89.Name, CPS::Logger::Level::off);
  line89EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(ieee9.line89.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.line89.Inductance),
      Math::singlePhaseParameterToThreePhase(ieee9.line89.Capacitance),
      Math::singlePhaseParameterToThreePhase(ieee9.line89.Conductance));

  // Transformers
  auto transf14EMT =
      EMT::Ph3::Transformer::make(ieee9.transf14.Name, CPS::Logger::Level::off);
  transf14EMT->setParameters(
      ieee9.transf14.VoltageLVSide, ieee9.transf14.VoltageHVSide,
      ieee9.transf14.RatedPower, ieee9.transf14.Ratio, 0.0,
      Math::singlePhaseParameterToThreePhase(ieee9.transf14.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.transf14.Inductance));

  auto transf27EMT =
      EMT::Ph3::Transformer::make(ieee9.transf27.Name, CPS::Logger::Level::off);
  transf27EMT->setParameters(
      ieee9.transf27.VoltageLVSide, ieee9.transf27.VoltageHVSide,
      ieee9.transf27.RatedPower, ieee9.transf27.Ratio, 0.0,
      Math::singlePhaseParameterToThreePhase(ieee9.transf27.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.transf27.Inductance));

  auto transf39EMT =
      EMT::Ph3::Transformer::make(ieee9.transf39.Name, CPS::Logger::Level::off);
  transf39EMT->setParameters(
      ieee9.transf39.VoltageLVSide, ieee9.transf39.VoltageHVSide,
      ieee9.transf39.RatedPower, ieee9.transf39.Ratio, 0.0,
      Math::singlePhaseParameterToThreePhase(ieee9.transf39.Resistance),
      Math::singlePhaseParameterToThreePhase(ieee9.transf39.Inductance));

  // Connect components to nodes
  gen1EMT->connect({n1EMT});
  // Inverter terminals: 0 = GND, 1 = PCC.
  gen2EMT->connect({SimNode<Real>::GND, n2EMT});
  gen3EMT->connect({SimNode<Real>::GND, n3EMT});

  load5EMT->connect({n5EMT});
  load6EMT->connect({n6EMT});
  load8EMT->connect({n8EMT});

  line54EMT->connect({n5EMT, n4EMT});
  line64EMT->connect({n6EMT, n4EMT});
  line75EMT->connect({n7EMT, n5EMT});
  line96EMT->connect({n9EMT, n6EMT});
  line78EMT->connect({n7EMT, n8EMT});
  line89EMT->connect({n8EMT, n9EMT});

  transf14EMT->connect({n1EMT, n4EMT});
  transf27EMT->connect({n2EMT, n7EMT});
  transf39EMT->connect({n3EMT, n9EMT});

  // Create system topology
  auto systemEMT = SystemTopology(
      ieee9.nomFreq,
      SystemNodeList{n1EMT, n2EMT, n3EMT, n4EMT, n5EMT, n6EMT, n7EMT, n8EMT,
                     n9EMT},
      SystemComponentList{gen1EMT, gen2EMT, gen3EMT, load5EMT, load6EMT,
                          load8EMT, line54EMT, line64EMT, line75EMT, line96EMT,
                          line78EMT, line89EMT, transf14EMT, transf27EMT,
                          transf39EMT});

  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  // Logger
  if (logger) {
    // Logging
    logger->logAttribute("BUS1", n1EMT->attribute("v"));
    logger->logAttribute("BUS2", n2EMT->attribute("v"));
    logger->logAttribute("BUS3", n3EMT->attribute("v"));
    logger->logAttribute("BUS4", n4EMT->attribute("v"));
    logger->logAttribute("BUS5", n5EMT->attribute("v"));
    logger->logAttribute("BUS6", n6EMT->attribute("v"));
    logger->logAttribute("BUS7", n7EMT->attribute("v"));
    logger->logAttribute("BUS8", n8EMT->attribute("v"));
    logger->logAttribute("BUS9", n9EMT->attribute("v"));

    // GFM inverter (gen2) signals
    logger->logAttribute("GEN2.I", gen2EMT->attribute("i_intf"));
    logger->logAttribute("GEN2.V", gen2EMT->attribute("v_intf"));
    logger->logAttribute("GEN2.p_inst", gen2EMT->attribute("p_inst"));
    logger->logAttribute("GEN2.q_inst", gen2EMT->attribute("q_inst"));
    logger->logAttribute("GEN2.omega", gen2EMT->attribute("omega_gfm"));
    logger->logAttribute("GEN2.vc_d", gen2EMT->attribute("vc_d"));
    logger->logAttribute("GEN2.vc_q", gen2EMT->attribute("vc_q"));

    // GFL inverter (gen3) signals
    logger->logAttribute("GEN3.I", gen3EMT->attribute("i_intf"));
    logger->logAttribute("GEN3.V", gen3EMT->attribute("v_intf"));
    logger->logAttribute("GEN3.p_inst", gen3EMT->attribute("p_inst"));
    logger->logAttribute("GEN3.q_inst", gen3EMT->attribute("q_inst"));
    logger->logAttribute("GEN3.omega_pll", gen3EMT->attribute("omega_pll"));
    logger->logAttribute("GEN3.vc_d", gen3EMT->attribute("vc_d"));
    logger->logAttribute("GEN3.vc_q", gen3EMT->attribute("vc_q"));

    // log generator's current
    for (auto comp : systemEMT.mComponents) {
      if (std::dynamic_pointer_cast<CPS::EMT::Ph3::SynchronGenerator4OrderVBR>(
              comp)) {
        logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
        logger->logAttribute(comp->name() + ".V", comp->attribute("v_intf"));
        logger->logAttribute(comp->name() + ".omega", comp->attribute("w_r"));
        logger->logAttribute(comp->name() + ".delta", comp->attribute("delta"));
      }
    }

    // log transfomers voltages & currents
    for (auto comp : systemEMT.mComponents) {
      if (std::dynamic_pointer_cast<CPS::EMT::Ph3::Transformer>(comp)) {
        logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
        logger->logAttribute(comp->name() + ".V", comp->attribute("v_intf"));
      }
    }

    // log Lines voltages & currents
    for (auto comp : systemEMT.mComponents) {
      if (std::dynamic_pointer_cast<CPS::EMT::Ph3::PiLine>(comp)) {
        logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
        logger->logAttribute(comp->name() + ".V", comp->attribute("v_intf"));
      }
    }
  }

  return systemEMT;
}

int main(int argc, char *argv[]) {

  CommandLineArgs args(argc, argv, "EMT_Ph3_IEEE9_SSN_InverterMix", 0.00005,
                       0.01 * 60, 60, -1, CPS::Logger::Level::info,
                       CPS::Logger::Level::off, false, false, false,
                       CPS::Domain::EMT);

  CPS::Logger::setLogDir("./logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() &&
             args.getOptionBool("log");

  std::filesystem::path logFilename =
      "./logs/" + args.name + "/" + args.name + ".csv";
  std::shared_ptr<DataLoggerInterface> logger = nullptr;

  if (log) {
    logger =
        RealTimeDataLogger::make(logFilename, args.duration, args.timeStep);
  }

  auto sys = buildTopology(args, logger);

  Simulation sim(args.name, args);
  sim.setSystem(sys);
  sim.setDomain(Domain::EMT);
  sim.doSystemMatrixRecomputation(true);
  if (log) {
    sim.addLogger(logger);
  }
  sim.run();

  CPS::Logger::get(args.name)->info("Simulation finished.");
}
