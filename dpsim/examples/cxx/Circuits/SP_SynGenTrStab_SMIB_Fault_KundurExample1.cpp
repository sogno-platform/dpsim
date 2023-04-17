/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "../Examples.h"
#include <DPsim.h>

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;
using namespace Examples::Grids;

// This example is taken from:
// P. Kundur, "Power System Stability and Control", Example 13.2, pp. 864-869.

// Network
KundurExample1::Network net;
Real nomVoltNetwork = net.nomVoltage;
Real nomFreq = 60;
Real nomOmega = nomFreq * 2 * PI;

// SG - general
KundurExample1::Gen gen;
Real syngenNomPower = gen.nomPower;
Real syngenNomVoltage = gen.nomVoltage;
Real syngenH = gen.H;
Real syngenXpdPU = gen.XpdPU;
Real syngenRsPU = gen.RsPU;
Real syngenD = gen.D;

// SG - init
Real initActivePower = 0.9 * syngenNomPower;
Real setPointVoltage = syngenNomVoltage;

// Transformer
KundurExample1::Transf1 transf1;
Real transformerNomVoltageHV = transf1.nomVoltageHV;
Real transformerNomVoltageMV = transf1.nomVoltageMV;
Real transformerResistance = transf1.transformerResistance;
Real transformerInductance = transf1.transformerReactance / nomOmega;
Real transformerRatio = transformerNomVoltageHV / transformerNomVoltageMV;

// Line
KundurExample1::Line1 line1;
Real lineResistance = line1.lineResistance;
Real lineInductance = line1.lineReactance / nomOmega;
Real lineCapacitance = line1.lineSusceptance / nomOmega;
Real lineConductance = line1.lineConductance;

// Slack
Real Vslack = 0.90081 * nomVoltNetwork;

// Switch to trigger fault at generator terminal
Real SwitchOpen = 1e12;
Real SwitchClosed = 7.21;

void SP_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime,
                               Real startTimeFault, Real endTimeFault,
                               Real cmdInertiaFactor) {
  // ----- POWERFLOW FOR INITIALIZATION -----
  Real timeStepPF = finalTime;
  Real finalTimePF = finalTime + timeStepPF;
  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  // Components
  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

  // Synchronous generator ideal model
  auto genPF = SP::Ph1::SynchronGenerator::make("SynGen", Logger::Level::debug);
  // setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
  genPF->setParameters(syngenNomPower, syngenNomVoltage, initActivePower,
                       setPointVoltage, PowerflowBusType::PV);
  genPF->setBaseVoltage(nomVoltNetwork);
  genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

  // Transformer
  auto trafoPF =
      CPS::SP::Ph1::Transformer::make("trafo", "trafo", Logger::Level::debug);
  trafoPF->setParameters(transformerNomVoltageHV, transformerNomVoltageMV,
                         transformerRatio, 0, transformerResistance,
                         transformerInductance);
  trafoPF->setBaseVoltage(nomVoltNetwork);

  // Slack
  auto extnetPF =
      SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
  extnetPF->setParameters(Vslack);
  extnetPF->setBaseVoltage(nomVoltNetwork);
  extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  // Line
  auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
  linePF->setParameters(lineResistance, lineInductance, lineCapacitance,
                        lineConductance);
  linePF->setBaseVoltage(nomVoltNetwork);

  // Switch
  auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
  faultPF->setParameters(SwitchOpen, SwitchClosed);
  faultPF->open();

  // Topology
  genPF->connect({n1PF});
  trafoPF->connect({n2PF, n1PF});
  faultPF->connect({SP::SimNode::GND, n2PF});
  linePF->connect({n2PF, n3PF});
  extnetPF->connect({n3PF});
  auto systemPF = SystemTopology(
      60, SystemNodeList{n1PF, n2PF, n3PF},
      SystemComponentList{genPF, trafoPF, linePF, extnetPF, faultPF});

  // Logging
  auto loggerPF = DataLogger::make(simNamePF);
  loggerPF->logAttribute("v1", n1PF->attribute("v"));
  loggerPF->logAttribute("v2", n2PF->attribute("v"));
  loggerPF->logAttribute("v3", n3PF->attribute("v"));

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

  // ----- Dynamic simulation ------
  String simNameSP = simName + "_SP";
  Logger::setLogDir("logs/" + simNameSP);

  // Nodes
  auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3SP = SimNode<Complex>::make("n3", PhaseType::Single);

  // Components
  // Synchronous generator
  auto genSP = CPS::SP::Ph1::SynchronGeneratorTrStab::make(
      "SynGen", Logger::Level::debug);

  // Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
  genSP->setStandardParametersPU(syngenNomPower, syngenNomVoltage, nomFreq,
                                 syngenXpdPU, cmdInertiaFactor * syngenH,
                                 syngenRsPU, syngenD);
  genSP->setModelFlags(false);

  // Transformer
  auto trafoSP =
      CPS::SP::Ph1::Transformer::make("trafo", "trafo", Logger::Level::debug);
  trafoSP->setParameters(transformerNomVoltageHV, transformerNomVoltageMV,
                         transformerRatio, 0, transformerResistance,
                         transformerInductance);

  // Slack
  auto extnetSP =
      SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
  extnetSP->setParameters(Vslack);

  // Line
  auto lineSP = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
  lineSP->setParameters(lineResistance, lineInductance, lineCapacitance,
                        lineConductance);

  //Switch
  auto faultSP = SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
  faultSP->setParameters(SwitchOpen, SwitchClosed);
  faultSP->open();

  // Topology
  genSP->connect({n1SP});
  trafoSP->connect({n2SP, n1SP});
  faultSP->connect({SP::SimNode::GND, n2SP});
  lineSP->connect({n2SP, n3SP});
  extnetSP->connect({n3SP});
  auto systemSP = SystemTopology(
      60, SystemNodeList{n1SP, n2SP, n3SP},
      SystemComponentList{genSP, trafoSP, lineSP, extnetSP, faultSP});

  // Initialization of dynamic topology
  systemSP.initWithPowerflow(systemPF, Domain::SP);

  // Logging
  auto loggerSP = DataLogger::make(simNameSP);
  loggerSP->logAttribute("v1", n1SP->attribute("v"));
  loggerSP->logAttribute("v2", n2SP->attribute("v"));
  loggerSP->logAttribute("v3", n3SP->attribute("v"));

  //gen
  loggerSP->logAttribute("Ep", genSP->attribute("Ep"));
  loggerSP->logAttribute("v_gen", genSP->attribute("v_intf"));
  loggerSP->logAttribute("i_gen", genSP->attribute("i_intf"));
  loggerSP->logAttribute("wr_gen", genSP->attribute("w_r"));
  loggerSP->logAttribute("delta_r_gen", genSP->attribute("delta_r"));
  loggerSP->logAttribute("P_elec", genSP->attribute("P_elec"));
  loggerSP->logAttribute("Q_elec", genSP->attribute("Q_elec"));
  loggerSP->logAttribute("P_mech", genSP->attribute("P_mech"));

  //Switch
  loggerSP->logAttribute("v_fault", faultSP->attribute("v_intf"));
  loggerSP->logAttribute("i_fault", faultSP->attribute("i_intf"));

  //line
  loggerSP->logAttribute("v_line", lineSP->attribute("v_intf"));
  loggerSP->logAttribute("i_line", lineSP->attribute("i_intf"));

  //slack
  loggerSP->logAttribute("v_slack", extnetSP->attribute("v_intf"));
  loggerSP->logAttribute("i_slack", extnetSP->attribute("i_intf"));

  Simulation simSP(simNameSP, Logger::Level::debug);
  simSP.setSystem(systemSP);
  simSP.setTimeStep(timeStep);
  simSP.setFinalTime(finalTime);
  simSP.setDomain(Domain::SP);
  simSP.addLogger(loggerSP);
  // simSP.doSystemMatrixRecomputation(true);

  // Events
  auto sw1 = SwitchEvent::make(std::round(startTimeFault / timeStep) * timeStep,
                               faultSP, true);
  simSP.addEvent(sw1);
  auto sw2 = SwitchEvent::make(std::round(endTimeFault / timeStep) * timeStep,
                               faultSP, false);
  simSP.addEvent(sw2);

  simSP.run();
}

int main(int argc, char *argv[]) {

  //Simultion parameters
  String simName = "SP_SynGenTrStab_SMIB_Fault";
  Real finalTime = 10;
  Real timeStep = 0.001;
  Real startTimeFault = 0.5;
  Real endTimeFault = 0.57;
  Real cmdInertiaFactor = 1.0;

  CommandLineArgs args(argc, argv);
  if (argc > 1) {
    timeStep = args.timeStep;
    if (args.name != "dpsim")
      simName = args.name;
    if (args.options.find("SCALEINERTIA") != args.options.end())
      cmdInertiaFactor = args.getOptionReal("SCALEINERTIA");
    if (args.options.find("STARTTIMEFAULT") != args.options.end())
      startTimeFault = args.getOptionReal("STARTTIMEFAULT");
    if (args.options.find("ENDTIMEFAULT") != args.options.end())
      endTimeFault = args.getOptionReal("ENDTIMEFAULT");
  }

  SP_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, startTimeFault,
                            endTimeFault, cmdInertiaFactor);
}
