// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "../Examples.h"
#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

int main(int argc, char *argv[]) {

  CIM::Examples::Components::GFM::Yazdani Yazdani;

  Real finalTime = 1.0;
  Real timeStep = 100e-6;
  String simName = "EMT_VSI_GFM_LoadStep";
  Bool pvWithControl = true;

  // ----- POWERFLOW FOR INITIALIZATION -----
  Real timeStepPF = finalTime;
  Real finalTimePF = finalTime + timeStepPF;
  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  // Components Powerflow Init
  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

  Complex load1_s =
      3 * std::pow(400, 2) / (Complex(83e-3, 137e-6 * 2 * M_PI * 60));
  Real load1_p = load1_s.real();
  Real load1_q = load1_s.imag();

  Complex load2_s =
      3 * std::pow(400, 2) /
      (Complex(Yazdani.Res2, Yazdani.Ind2 * 2 * M_PI * 60 -
                                 1 / (2 * M_PI * 60 * Yazdani.Cap2)));
  Real load2_p = load2_s.real();
  Real load2_q = load2_s.imag();

  auto extnetPF =
      SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
  extnetPF->setParameters(400);
  extnetPF->setBaseVoltage(400);
  extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
  linePF->setParameters(0.88e-3, 0, 0);
  linePF->setBaseVoltage(400);

  // Topology
  extnetPF->connect({n1PF});
  linePF->connect({n1PF, n2PF});

  auto systemPF = SystemTopology(60, SystemNodeList{n1PF, n2PF},
                                 SystemComponentList{linePF, extnetPF});

  // Logging
  auto loggerPF = DataLogger::make(simNamePF);
  loggerPF->logAttribute("v1", n1PF->attribute("v"));
  loggerPF->logAttribute("v2", n2PF->attribute("v"));

  // Simulation
  Simulation simPF(simNamePF, Logger::Level::debug);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(timeStepPF);
  simPF.setFinalTime(finalTimePF);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(false);
  simPF.addLogger(loggerPF);
  simPF.run();

  // ----- DYNAMIC EMT SIMULATION -----
  Real timeStepEMT = timeStep;
  Real finalTimeEMT = finalTime + timeStepEMT;
  String simNameEMT = simName + "_EMT";
  Logger::setLogDir("logs/" + simNameEMT);

  auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
  auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
  auto n3EMT = SimNode<Real>::make("n3", PhaseType::ABC);
  auto n4EMT = SimNode<Real>::make("n4", PhaseType::ABC);

  auto loadEMT1 = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
  loadEMT1->setParameters(CPS::Math::singlePhasePowerToThreePhase(load1_p),
                          CPS::Math::singlePhasePowerToThreePhase(load1_q),
                          400);

  auto loadEMT2 = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
  loadEMT2->setParameters(CPS::Math::singlePhasePowerToThreePhase(load2_p),
                          CPS::Math::singlePhasePowerToThreePhase(load2_q),
                          400);

  auto resOnEMT = EMT::Ph3::Resistor::make("ResOn", Logger::Level::debug);
  resOnEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(0.88e-3));

  auto pv = EMT::Ph3::VSIVoltageControlVCO::make("pv", "pv",
                                                 Logger::Level::debug, false);
  pv->setParameters(Yazdani.OmegaNull, Yazdani.Vdref, Yazdani.Vqref);
  pv->setControllerParameters(Yazdani.KpVoltageCtrl, Yazdani.KiVoltageCtrl,
                              Yazdani.KpCurrCtrl, Yazdani.KiCurrCtrl,
                              Yazdani.OmegaNull);
  pv->setFilterParameters(Yazdani.Lf, Yazdani.Cf, Yazdani.Rf, Yazdani.Rc);
  pv->setInitialStateValues(Yazdani.phi_dInit, Yazdani.phi_qInit,
                            Yazdani.gamma_dInit, Yazdani.gamma_qInit);
  pv->withControl(pvWithControl);

  // Fault Event
  Real SwitchOpen = 1e9;
  Real SwitchClosed = 1e-9;

  auto fault1EMT = CPS::EMT::Ph3::Switch::make("Switch1", Logger::Level::debug);
  fault1EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(SwitchOpen),
      Math::singlePhaseParameterToThreePhase(SwitchClosed));
  fault1EMT->openSwitch();

  auto fault2EMT = CPS::EMT::Ph3::Switch::make("Switch2", Logger::Level::debug);
  fault2EMT->setParameters(
      Math::singlePhaseParameterToThreePhase(SwitchOpen),
      Math::singlePhaseParameterToThreePhase(SwitchClosed));
  fault2EMT->openSwitch();

  // Topology
  pv->connect({n1EMT});
  resOnEMT->connect({n1EMT, n2EMT});
  fault1EMT->connect({n2EMT, n3EMT});
  loadEMT1->connect({n3EMT});
  fault2EMT->connect({n3EMT, n4EMT});
  loadEMT2->connect({n4EMT});

  auto systemEMT =
      SystemTopology(60, SystemNodeList{n1EMT, n2EMT, n3EMT, n4EMT},
                     SystemComponentList{resOnEMT, fault1EMT, fault2EMT,
                                         loadEMT1, loadEMT2, pv});

  // Initialization of dynamic topology
  systemEMT.initWithPowerflow(systemPF, CPS::Domain::EMT);

  Complex initial3PhPowerVSI =
      Complex(linePF->attributeTyped<Real>("p_inj")->get(),
              linePF->attributeTyped<Real>("q_inj")->get());

  pv->terminal(0)->setPower(initial3PhPowerVSI);

  // Logging
  auto loggerEMT = DataLogger::make(simNameEMT);
  loggerEMT->logAttribute("Voltage_PCC", n1EMT->attribute("v"));
  loggerEMT->logAttribute("Voltage_Node_2", n2EMT->attribute("v"));
  loggerEMT->logAttribute("Voltage_Node_3", n3EMT->attribute("v"));
  loggerEMT->logAttribute("Voltage_Source", pv->attribute("Vs"));
  loggerEMT->logAttribute("Current_RLC", pv->attribute("i_intf"));
  loggerEMT->logAttribute("P_elec", pv->attribute("P_elec"));
  loggerEMT->logAttribute("Q_elec", pv->attribute("Q_elec"));

  // Fault Event Timers
  Real startTimeFault1 = 0.2;
  Real endTimeFault1 = 0.8;
  Real startTimeFault2 = 0.4;
  Real endTimeFault2 = 0.6;

  // Simulation
  Simulation sim(simNameEMT, Logger::Level::debug);

  auto sw1close = SwitchEvent3Ph::make(startTimeFault1, fault1EMT, true);
  sim.addEvent(sw1close);
  auto sw1open = SwitchEvent3Ph::make(endTimeFault1, fault1EMT, false);
  sim.addEvent(sw1open);

  auto sw2close = SwitchEvent3Ph::make(startTimeFault2, fault2EMT, true);
  sim.addEvent(sw2close);
  auto sw2open = SwitchEvent3Ph::make(endTimeFault2, fault2EMT, false);
  sim.addEvent(sw2open);

  sim.setSystem(systemEMT);
  sim.setTimeStep(timeStepEMT);
  sim.setFinalTime(finalTimeEMT);
  sim.setDomain(Domain::EMT);
  sim.addLogger(loggerEMT);
  sim.run();
}
