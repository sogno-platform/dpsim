// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "../Examples.h"
#include "../GeneratorFactory.h"

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

SystemTopology buildTopology(CommandLineArgs &args,
                             std::shared_ptr<DataLoggerInterface> logger) {

  String simName = args.name;

  CPS::CIM::Examples::Grids::IEEE9::ScenarioConfig ieee9(args.sysFreq);

  // Power flow simulation
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

  auto gen3PF = SP::Ph1::SynchronGenerator::make(ieee9.gen3.Name,
                                                 CPS::Logger::Level::off);
  gen3PF->setParameters(ieee9.gen3.RatedPower, ieee9.gen3.RatedVoltage,
                        ieee9.gen3.InitialPower, ieee9.gen3.InitialVoltage,
                        ieee9.gen3.BusType);
  gen3PF->setBaseVoltage(ieee9.gen3.RatedVoltage);

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
  transf14PF->setParameters(ieee9.transf14.VoltageLVSide,
                            ieee9.transf14.VoltageHVSide, ieee9.transf14.Ratio,
                            0.0, ieee9.transf14.Resistance,
                            ieee9.transf14.Inductance);
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

  // Connect components to nodes

  gen1PF->connect({n1PF});
  gen2PF->connect({n2PF});
  gen3PF->connect({n3PF});

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

  // Create system topology for power flow
  auto systemPF = SystemTopology(
      ieee9.nomFreq,
      SystemNodeList{n1PF, n2PF, n3PF, n4PF, n5PF, n6PF, n7PF, n8PF, n9PF},
      SystemComponentList{gen1PF, gen2PF, gen3PF, load5PF, load6PF, load8PF,
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

  // DYNAMIC SIMULATION - SP

  String simNameSP = simName + "_SP";
  CPS::Logger::setLogDir("logs/" + simNameSP);

  // Nodes
  auto n1SP = SimNode<Complex>::make("BUS1", PhaseType::Single);
  auto n2SP = SimNode<Complex>::make("BUS2", PhaseType::Single);
  auto n3SP = SimNode<Complex>::make("BUS3", PhaseType::Single);
  auto n4SP = SimNode<Complex>::make("BUS4", PhaseType::Single);
  auto n5SP = SimNode<Complex>::make("BUS5", PhaseType::Single);
  auto n6SP = SimNode<Complex>::make("BUS6", PhaseType::Single);
  auto n7SP = SimNode<Complex>::make("BUS7", PhaseType::Single);
  auto n8SP = SimNode<Complex>::make("BUS8", PhaseType::Single);
  auto n9SP = SimNode<Complex>::make("BUS9", PhaseType::Single);

  // Generator 1 Initialization
  auto gen1SP = SP::Ph1::SynchronGenerator4OrderVBR::make(
      ieee9.gen1.Name, CPS::Logger::Level::off);

  gen1SP->setOperationalParametersPerUnit(
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
  gen1SP->addExciter(exciter1);

  CPS::Real T4 = 1.0;
  CPS::Real T5 = 1.0;

  std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernor1 =
      Signal::TurbineGovernorType1::make("Gen1_TurbineGovernor",
                                         CPS::Logger::Level::off);

  turbineGovernor1->setParameters(ieee9.gov1.T2, T4, T5, ieee9.gov1.T3,
                                  ieee9.gov1.T1, ieee9.gov1.R, ieee9.gov1.Vmin,
                                  ieee9.gov1.Vmax, 1.0);

  gen1SP->addGovernor(turbineGovernor1);

  auto gen2SP = SP::Ph1::SynchronGenerator4OrderVBR::make(
      ieee9.gen2.Name, CPS::Logger::Level::off);

  gen2SP->setOperationalParametersPerUnit(
      ieee9.gen2.RatedPower,   // nomPower [VA]
      ieee9.gen2.RatedVoltage, // nomVolt [V]
      ieee9.nomFreq,           // nomFreq [Hz]
      ieee9.gen2.H, ieee9.gen2.Xd, ieee9.gen2.Xq, ieee9.gen2.Xa,
      ieee9.gen2.XdPrime, ieee9.gen2.XqPrime, ieee9.gen2.TdoPrime,
      ieee9.gen2.TqoPrime);

  auto exciter2Params = std::make_shared<Signal::ExciterDC1SimpParameters>();
  exciter2Params->Ta = ieee9.exc2.TA;
  exciter2Params->Ka = ieee9.exc2.KA;
  exciter2Params->Tef = ieee9.exc2.TE;
  exciter2Params->Kef = ieee9.exc2.KE;
  exciter2Params->Tf = ieee9.exc2.TF;
  exciter2Params->Kf = ieee9.exc2.KF;
  exciter2Params->Tr = 0.01;
  exciter2Params->MaxVa = ieee9.exc2.VRmax;
  exciter2Params->MinVa = ieee9.exc2.VRmin;
  exciter2Params->Bef = std::log(ieee9.exc2.S_EX2 / ieee9.exc2.S_EX1) /
                        (ieee9.exc2.EX2 - ieee9.exc2.EX1);
  exciter2Params->Aef =
      ieee9.exc2.S_EX1 / std::exp(exciter2Params->Bef * ieee9.exc2.EX1);
  auto exciter2 =
      Signal::ExciterDC1Simp::make("Gen2_Exciter", CPS::Logger::Level::off);
  exciter2->setParameters(exciter2Params);
  gen2SP->addExciter(exciter2);

  std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernor2 =
      Signal::TurbineGovernorType1::make("Gen2_TurbineGovernor",
                                         CPS::Logger::Level::off);

  turbineGovernor2->setParameters(ieee9.gov2.T2, T4, T5, ieee9.gov2.T3,
                                  ieee9.gov2.T1, ieee9.gov2.R, ieee9.gov2.Vmin,
                                  ieee9.gov2.Vmax, 1.0);

  gen2SP->addGovernor(turbineGovernor2);

  auto gen3SP = SP::Ph1::SynchronGenerator4OrderVBR::make(
      ieee9.gen3.Name, CPS::Logger::Level::off);

  gen3SP->setOperationalParametersPerUnit(
      ieee9.gen3.RatedPower,   // nomPower [VA]
      ieee9.gen3.RatedVoltage, // nomVolt [V]
      ieee9.nomFreq,           // nomFreq [Hz]
      ieee9.gen3.H, ieee9.gen3.Xd, ieee9.gen3.Xq, ieee9.gen3.Xa,
      ieee9.gen3.XdPrime, ieee9.gen3.XqPrime, ieee9.gen3.TdoPrime,
      ieee9.gen3.TqoPrime);

  auto exciter3Params = std::make_shared<Signal::ExciterDC1SimpParameters>();
  exciter3Params->Ta = ieee9.exc3.TA;
  exciter3Params->Ka = ieee9.exc3.KA;
  exciter3Params->Tef = ieee9.exc3.TE;
  exciter3Params->Kef = ieee9.exc3.KE;
  exciter3Params->Tf = ieee9.exc3.TF;
  exciter3Params->Kf = ieee9.exc3.KF;
  exciter3Params->Tr = 0.01;
  exciter3Params->MaxVa = ieee9.exc3.VRmax;
  exciter3Params->MinVa = ieee9.exc3.VRmin;
  exciter3Params->Bef = std::log(ieee9.exc3.S_EX2 / ieee9.exc3.S_EX1) /
                        (ieee9.exc3.EX2 - ieee9.exc3.EX1);
  exciter3Params->Aef =
      ieee9.exc3.S_EX1 / std::exp(exciter3Params->Bef * ieee9.exc3.EX1);
  auto exciter3 =
      Signal::ExciterDC1Simp::make("Gen3_Exciter", CPS::Logger::Level::off);
  exciter3->setParameters(exciter3Params);
  gen3SP->addExciter(exciter3);

  std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernor3 =
      Signal::TurbineGovernorType1::make("Gen3_TurbineGovernor",
                                         CPS::Logger::Level::off);

  turbineGovernor3->setParameters(ieee9.gov3.T2, T4, T5, ieee9.gov3.T3,
                                  ieee9.gov3.T1, ieee9.gov3.R, ieee9.gov3.Vmin,
                                  ieee9.gov3.Vmax, 1.0);

  gen3SP->addGovernor(turbineGovernor3);

  // Loads
  auto load5SP = SP::Ph1::Load::make(ieee9.load5.Name, CPS::Logger::Level::off);
  load5SP->setParameters(ieee9.load5.RealPower, ieee9.load5.ReactivePower,
                         ieee9.load5.BaseVoltage);

  auto load6SP = SP::Ph1::Load::make(ieee9.load6.Name, CPS::Logger::Level::off);
  load6SP->setParameters(ieee9.load6.RealPower, ieee9.load6.ReactivePower,
                         ieee9.load6.BaseVoltage);

  auto load8SP = SP::Ph1::Load::make(ieee9.load8.Name, CPS::Logger::Level::off);
  load8SP->setParameters(ieee9.load8.RealPower, ieee9.load8.ReactivePower,
                         ieee9.load8.BaseVoltage);

  // Lines
  auto line54SP =
      SP::Ph1::PiLine::make(ieee9.line54.Name, CPS::Logger::Level::off);
  line54SP->setParameters(ieee9.line54.Resistance, ieee9.line54.Inductance,
                          ieee9.line54.Capacitance, ieee9.line54.Conductance);

  auto line64SP =
      SP::Ph1::PiLine::make(ieee9.line64.Name, CPS::Logger::Level::off);
  line64SP->setParameters(ieee9.line64.Resistance, ieee9.line64.Inductance,
                          ieee9.line64.Capacitance, ieee9.line64.Conductance);

  auto line75SP =
      SP::Ph1::PiLine::make(ieee9.line75.Name, CPS::Logger::Level::off);
  line75SP->setParameters(ieee9.line75.Resistance, ieee9.line75.Inductance,
                          ieee9.line75.Capacitance, ieee9.line75.Conductance);

  auto line96SP =
      SP::Ph1::PiLine::make(ieee9.line96.Name, CPS::Logger::Level::off);
  line96SP->setParameters(ieee9.line96.Resistance, ieee9.line96.Inductance,
                          ieee9.line96.Capacitance, ieee9.line96.Conductance);

  auto line78SP =
      SP::Ph1::PiLine::make(ieee9.line78.Name, CPS::Logger::Level::off);
  line78SP->setParameters(ieee9.line78.Resistance, ieee9.line78.Inductance,
                          ieee9.line78.Capacitance, ieee9.line78.Conductance);

  auto line89SP =
      SP::Ph1::PiLine::make(ieee9.line89.Name, CPS::Logger::Level::off);
  line89SP->setParameters(ieee9.line89.Resistance, ieee9.line89.Inductance,
                          ieee9.line89.Capacitance, ieee9.line89.Conductance);

  // Transformers
  auto transf14SP =
      SP::Ph1::Transformer::make(ieee9.transf14.Name, CPS::Logger::Level::off);
  transf14SP->setParameters(
      ieee9.transf14.VoltageLVSide, ieee9.transf14.VoltageHVSide,
      ieee9.transf14.RatedPower, ieee9.transf14.Ratio, 0.0,
      ieee9.transf14.Resistance, ieee9.transf14.Inductance);

  auto transf27SP =
      SP::Ph1::Transformer::make(ieee9.transf27.Name, CPS::Logger::Level::off);
  transf27SP->setParameters(
      ieee9.transf27.VoltageLVSide, ieee9.transf27.VoltageHVSide,
      ieee9.transf27.RatedPower, ieee9.transf27.Ratio, 0.0,
      ieee9.transf27.Resistance, ieee9.transf27.Inductance);

  auto transf39SP =
      SP::Ph1::Transformer::make(ieee9.transf39.Name, CPS::Logger::Level::off);
  transf39SP->setParameters(
      ieee9.transf39.VoltageLVSide, ieee9.transf39.VoltageHVSide,
      ieee9.transf39.RatedPower, ieee9.transf39.Ratio, 0.0,
      ieee9.transf39.Resistance, ieee9.transf39.Inductance);

  // Connect components to nodes
  gen1SP->connect({n1SP});
  gen2SP->connect({n2SP});
  gen3SP->connect({n3SP});

  load5SP->connect({n5SP});
  load6SP->connect({n6SP});
  load8SP->connect({n8SP});

  line54SP->connect({n5SP, n4SP});
  line64SP->connect({n6SP, n4SP});
  line75SP->connect({n7SP, n5SP});
  line96SP->connect({n9SP, n6SP});
  line78SP->connect({n7SP, n8SP});
  line89SP->connect({n8SP, n9SP});

  transf14SP->connect({n1SP, n4SP});
  transf27SP->connect({n2SP, n7SP});
  transf39SP->connect({n3SP, n9SP});

  // Create system topology
  auto systemSP = SystemTopology(
      ieee9.nomFreq,
      SystemNodeList{n1SP, n2SP, n3SP, n4SP, n5SP, n6SP, n7SP, n8SP, n9SP},
      SystemComponentList{gen1SP, gen2SP, gen3SP, load5SP, load6SP, load8SP,
                          line54SP, line64SP, line75SP, line96SP, line78SP,
                          line89SP, transf14SP, transf27SP, transf39SP});

  systemSP.initWithPowerflow(systemPF, Domain::SP);

  // Logger
  if (logger) {
    // Logging
    logger->logAttribute("BUS1", n1SP->attribute("v"));
    logger->logAttribute("BUS2", n2SP->attribute("v"));
    logger->logAttribute("BUS3", n3SP->attribute("v"));
    logger->logAttribute("BUS4", n4SP->attribute("v"));
    logger->logAttribute("BUS5", n5SP->attribute("v"));
    logger->logAttribute("BUS6", n6SP->attribute("v"));
    logger->logAttribute("BUS7", n7SP->attribute("v"));
    logger->logAttribute("BUS8", n8SP->attribute("v"));
    logger->logAttribute("BUS9", n9SP->attribute("v"));

    // log generator's current
    for (auto comp : systemSP.mComponents) {
      if (std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator4OrderVBR>(
              comp)) {
        logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
        logger->logAttribute(comp->name() + ".V", comp->attribute("v_intf"));
        logger->logAttribute(comp->name() + ".omega", comp->attribute("w_r"));
        logger->logAttribute(comp->name() + ".delta", comp->attribute("delta"));
      }
    }

    // log transfomers voltages & currents
    for (auto comp : systemSP.mComponents) {
      if (std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp)) {
        logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
        logger->logAttribute(comp->name() + ".V", comp->attribute("v_intf"));
      }
    }

    // log Lines voltages & currents
    for (auto comp : systemSP.mComponents) {
      if (std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp)) {
        logger->logAttribute(comp->name() + ".I", comp->attribute("i_intf"));
        logger->logAttribute(comp->name() + ".V", comp->attribute("v_intf"));
      }
    }
  }

  return systemSP;
}

int main(int argc, char *argv[]) {
  CommandLineArgs args(argc, argv, "SP_Ph1_IEEE9_4Order", 0.00005, 0.01 * 60,
                       60, -1, CPS::Logger::Level::info,
                       CPS::Logger::Level::off, false, false, false,
                       CPS::Domain::SP);
  std::error_code ec;
  std::filesystem::create_directories("./logs", ec);

  CPS::Logger::setLogDir("./logs/" + args.name);
  bool log = args.options.find("log") != args.options.end() &&
             args.getOptionBool("log");

  std::filesystem::path logFilename = "./logs/" + args.name + ".csv";
  std::shared_ptr<DataLoggerInterface> logger = nullptr;

  if (log) {
    logger =
        RealTimeDataLogger::make(logFilename, args.duration, args.timeStep);
  }

  auto sys = buildTopology(args, logger);

  Simulation sim(args.name, args);
  sim.setSystem(sys);
  sim.setDomain(Domain::SP);
  sim.setSolverType(Solver::Type::MNA);
  sim.doSystemMatrixRecomputation(true);
  if (log) {
    sim.addLogger(logger);
  }
  sim.run();

  CPS::Logger::get("SP-IEEE9-4Order")->info("Simulation finished.");
}
