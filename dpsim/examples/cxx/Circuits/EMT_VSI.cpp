/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;

void EMT_Ph3_VSI2_4bus_SampleGrid() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.6;
  String simName = "EMT_Ph3_VSI2_4BUS_SampleGrid";
  Logger::setLogDir("logs/" + simName);
  Real Vnom = 20e3;
  Real Vnom_pv = 2e3;
  // Nodes
  std::vector<Complex> initialVoltage{Complex(Vnom, 0), Complex(Vnom, 0),
                                      Complex(Vnom, 0)};
  std::vector<Complex> initialVoltage_DG{
      Complex(Vnom_pv, 0), Complex(Vnom_pv, 0), Complex(Vnom_pv, 0)};

  auto n1 = SimNode::make("n1", PhaseType::ABC, initialVoltage);
  auto n2 = SimNode::make("n2", PhaseType::ABC, initialVoltage);
  auto n3 = SimNode::make("n3", PhaseType::ABC, initialVoltage);
  auto n4 = SimNode::make("n4", PhaseType::ABC, initialVoltage);
  auto n5 = SimNode::make("n5", PhaseType::ABC, initialVoltage);
  auto n6 = SimNode::make("n6", PhaseType::ABC, initialVoltage);
  auto n7 = SimNode::make("n7", PhaseType::ABC, initialVoltage);
  auto n_trans1 = SimNode::make("n_trans1", PhaseType::ABC, initialVoltage_DG);
  auto n_trans2 = SimNode::make("n_trans2", PhaseType::ABC, initialVoltage_DG);

  // Components
  auto vsi = AvVoltageSourceInverterDQ::make("vsi", Logger::Level::info);
  Real Pref1 = 300000;
  Real Qref1 = 200;
  Real Pref2 = 400000;
  Real Qref2 = 1000;

  auto vs = VoltageSource::make("vs", Logger::Level::info);
  vs->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(Complex(Vnom, 0)), 50);
  auto vsi2 = AvVoltageSourceInverterDQ::make("vsi2", Logger::Level::debug);
  auto trans_DG1 = Ph3::Transformer::make("trans_pv1", Logger::Level::debug);
  auto trans_DG2 = Ph3::Transformer::make("trans_pv2", Logger::Level::debug);

  Real rf_param = 0.01;
  Real lf_param = 0.928e-3;
  Real cf_param = 789.3e-6;
  Real rc_param = 0.5;

  //Real transformerNominalPower = 5e6;

  trans_DG1->setParameters(Vnom_pv, Vnom, Vnom_pv / Vnom, 0.,
                           Matrix::Identity(3, 3) * rc_param,
                           Matrix::Identity(3, 3) * lf_param * 0.01);
  trans_DG2->setParameters(Vnom_pv, Vnom, Vnom_pv / Vnom, 0.,
                           Matrix::Identity(3, 3) * rc_param,
                           Matrix::Identity(3, 3) * lf_param * 0.01);

  //trans_inductor->setParameters(lf_param*0.01);
  auto piline = PiLine::make("piline", Logger::Level::debug);
  Matrix rline_param = Matrix::Zero(3, 3);
  rline_param << 0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04;

  piline->setParameters(rline_param * 10,
                        Matrix::Identity(3, 3) * lf_param * 0.0001,
                        Matrix::Identity(3, 3) * cf_param);

  auto rline2 = Resistor::make("rline2", Logger::Level::info);
  rline2->setParameters(rline_param);

  auto rline3 = Resistor::make("rline3", Logger::Level::info);
  rline3->setParameters(rline_param);

  auto rline4 = Resistor::make("rline4", Logger::Level::info);
  rline4->setParameters(rline_param);

  auto rload = Resistor::make("rload", Logger::Level::info);
  Matrix rload_param_1kw_220v = Matrix::Zero(3, 3);

  rload_param_1kw_220v << 47.6568, 0, 0, 0, 47.6568, 0, 0, 0, 47.6568;

  rload->setParameters(rload_param_1kw_220v);

  auto rload2 = Resistor::make("rload2", Logger::Level::info);
  rload2->setParameters(rload_param_1kw_220v);

  auto Lload = Inductor::make("Lload", Logger::Level::info);
  Matrix Lload_param_100var_220v = Matrix(3, 3);
  Lload_param_100var_220v << 0.01516, 0, 0, 0, 0.01516, 0, 0, 0, 0.01516;

  Lload->setParameters(Lload_param_100var_220v);

  auto Lload2 = Inductor::make("Lload2", Logger::Level::info);
  Lload2->setParameters(Lload_param_100var_220v);

  vsi->setParameters(2 * M_PI * 50, Vnom_pv, Pref1, Qref1);
  vsi->setControllerParameters(0.25, 2, 0.001, 0.08, 3.77, 1400, 2 * M_PI * 50);
  vsi->setFilterParameters(lf_param, cf_param, rf_param, rc_param);

  vsi2->setParameters(2 * M_PI * 50, Vnom_pv, Pref2, Qref2);
  vsi2->setControllerParameters(0.25, 2, 0.001, 0.08, 3.77, 1400,
                                2 * M_PI * 50);
  vsi2->setFilterParameters(lf_param, cf_param, rf_param, rc_param);

  Matrix p_load = Matrix::Zero(3, 3);
  p_load << 1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6;
  Matrix q_load = Matrix::Zero(3, 3);
  q_load << 1e2, 0, 0, 0, 1e2, 0, 0, 0, 1e2;

  auto rxLoad =
      RXLoad::make("rxload", p_load, q_load, 20e3, Logger::Level::info);

  // 4Bus case study
  vsi->connect(SimNode::List{n_trans1});
  trans_DG1->connect(SimNode::List{n_trans1, n1});
  //trans_inductor->connect(SimNode::List{ n_trans1, n1 });
  rxLoad->connect(SimNode::List{n1});
  vs->connect(SimNode::List{SimNode::GND, n2});
  piline->connect(SimNode::List{n1, n2});

  rline2->connect(SimNode::List{n1, n3});

  rload->connect(SimNode::List{n3, n6});
  Lload->connect(SimNode::List{n6, SimNode::GND});

  rline3->connect(SimNode::List{n3, n4});

  vsi2->connect(SimNode::List{n_trans2});
  trans_DG2->connect(SimNode::List{n_trans2, n4});

  rline4->connect(SimNode::List{n3, n5});
  rload2->connect(SimNode::List{n5, n7});
  Lload2->connect(SimNode::List{n7, SimNode::GND});

  // Define system topology
  auto sys = SystemTopology(
      50, SystemNodeList{n1, n2, n3, n4, n5, n6, n7, n_trans1, n_trans2},
      SystemComponentList{vsi, vs, piline, rline2, rline3, rline4, rload,
                          rload2, Lload, Lload2, vsi2, trans_DG1, trans_DG2,
                          rxLoad});

  // extract initial voltages to EMT system
  //NRpolarSolver nrpSolver(simName + ".pf", sysPF, 1, Domain::Static, Logger::Level::off);
  //nrpSolver.writePFResultsToDynamicTopology(sysPF, sys);

  // Logging
  //auto logger = DataLogger::make(simName);
  auto logger_vsi = DataLogger::make(simName + "_pv1");
  auto logger_vsi2 = DataLogger::make(simName + "_pv2");
  //// currents
  //logger->logAttribute("i_vs", vs->attribute("i_intf"));
  //logger->logAttribute("i_vsi", vsi->attribute("i_intf"));
  //logger->logAttribute("i_vsi2", vsi2->attribute("i_intf"));
  //logger->logAttribute("iload", rload->attribute("i_intf"));
  //logger->logAttribute("iqload", Lload->attribute("i_intf"));
  //// nodal voltages
  //logger->logAttribute("v_slack", n2->attribute("v"));
  //logger->logAttribute("vBus1", n1->attribute("v"));
  //logger->logAttribute("vBus2", n3->attribute("v"));
  //logger->logAttribute("vBus3", n5->attribute("v"));
  //// power outputs
  logger_vsi->logAttribute("P", vsi->attribute("p"));
  logger_vsi->logAttribute("Q", vsi->attribute("q"));
  logger_vsi->logAttribute("P_ref", vsi->attribute("P_ref"));
  logger_vsi->logAttribute("Q_ref", vsi->attribute("Q_ref"));
  //// vsi
  logger_vsi->logAttribute("Vcabc", vsi->attribute("Vcabc"));
  logger_vsi->logAttribute("Vcdq", vsi->attribute("Vcdq"));
  logger_vsi->logAttribute("Vsdq", vsi->attribute("Vsdq"));
  logger_vsi->logAttribute("ifdq", vsi->attribute("ifdq"));
  logger_vsi->logAttribute("igdq", vsi->attribute("igdq"));

  logger_vsi2->logAttribute("P", vsi2->attribute("p"));
  logger_vsi->logAttribute("Q", vsi2->attribute("q"));
  logger_vsi2->logAttribute("P_ref", vsi2->attribute("P_ref"));
  logger_vsi->logAttribute("Q_ref", vsi2->attribute("Q_ref"));
  //// vsi
  logger_vsi2->logAttribute("Vcabc", vsi2->attribute("Vcabc"));
  logger_vsi2->logAttribute("Vcdq", vsi2->attribute("Vcdq"));
  logger_vsi2->logAttribute("Vsdq", vsi2->attribute("Vsdq"));
  logger_vsi2->logAttribute("ifdq", vsi2->attribute("ifdq"));
  logger_vsi2->logAttribute("igdq", vsi2->attribute("igdq"));

  //// states of controller
  //logger->logAttribute("theta", vsi->attribute("theta"));
  //logger->logAttribute("phi_pll", vsi->attribute("phipll"));
  //logger->logAttribute("phid", vsi->attribute("phid"));
  //logger->logAttribute("phiq", vsi->attribute("phiq"));
  //logger->logAttribute("gammad", vsi->attribute("gammad"));
  //logger->logAttribute("gammaq", vsi->attribute("gammaq"));
  //// frequency
  logger_vsi->logAttribute("freq_vsi1", vsi->attribute("freq"));
  //logger->logAttribute("freq_vsi2", vsi2->attribute("freq"));
  //// output voltages
  //logger->logAttribute("vsi", vsi->attribute("v_intf"));
  //logger->logAttribute("vsi_dq", vsi->attribute("Vsdq"));
  //logger->logAttribute("vsi2", vsi2->attribute("v_intf"));
  //logger->logAttribute("vsi2_dq", vsi2->attribute("Vsdq"));

  Simulation sim(simName);
  sim.setSystem(sys);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::EMT);
  //sim.addLogger(logger);
  sim.addLogger(logger_vsi);
  sim.addLogger(logger_vsi2);
  sim.run();
}

int main(int argc, char *argv[]) { EMT_Ph3_VSI2_4bus_SampleGrid(); }
