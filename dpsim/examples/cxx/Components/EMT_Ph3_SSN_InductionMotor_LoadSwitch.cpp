// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

class EMT_Ph3_SSN_InductionMotor_Circuits {
public:
  EMT_Ph3_SSN_InductionMotor_Circuits() = default;
  ~EMT_Ph3_SSN_InductionMotor_Circuits() = default;

  void ColdStart_LoadSwitch();

private:
  String simName_coldStart = "EMT_Ph3_SSN_InductionMotor_ColdStart_LoadSwitch";

  static constexpr Real nominalFrequency = 50.0;
  static constexpr Int polePairs = 2;

  static constexpr Real Rs = 2.0; // stator resistance [ohm]
  static constexpr Real Rr = 1.5; // rotor resistance, referred to stator [ohm]

  static constexpr Real Lm = 0.15;     // mutual inductance [H]
  static constexpr Real Lls = 0.01;    // stator leakage [H]
  static constexpr Real Llr = 0.01;    // rotor leakage, referred to stator [H]
  static constexpr Real Ls = Lm + Lls; // total stator inductance [H]
  static constexpr Real Lr_dash =
      Lm + Llr; // total rotor inductance, referred to stator [H]

  static constexpr Real we = 2.0 * PI * nominalFrequency;
  static constexpr Real synchronousSpeed = we / static_cast<Real>(polePairs);
  static constexpr Real nominalMechanicalTorque = 1.0; // [N m]

  static constexpr Real rotorInertia = 0.05; // [kg m^2]
  static constexpr Real mechanicalDamping =
      nominalMechanicalTorque / synchronousSpeed; // Tload = Tnom * wm / wm_nom

  static constexpr Real Vamp =
      325.0; // peak phase voltage [V] (~230 V RMS line-to-neutral)

  static constexpr Real stepTime = 1e-4;
  static constexpr Real finalTime = 2.5;
  static constexpr Real switchTime = 1.0;
  static constexpr Real switchOpenRes = 1e6;
  static constexpr Real switchClosedRes = 1e-4;
};

void EMT_Ph3_SSN_InductionMotor_Circuits::ColdStart_LoadSwitch() {

  //Comps and nodes
  auto motor = EMT::Ph3::SSN_InductionMotor::make("motor_coldStart");
  motor->setParameters(nominalFrequency, polePairs, Rs, Rr, Ls, Lr_dash, Lm,
                       rotorInertia, mechanicalDamping,
                       -nominalMechanicalTorque, 0.0, false);

  auto vs = EMT::Ph3::VoltageSource::make("Vs");
  vs->setParameters(Math::singlePhaseVariableToThreePhase(Complex(Vamp, 0.0)));

  auto r_v = EMT::Ph3::Resistor::make("r_v");
  r_v->setParameters(0.05 * Matrix::Identity(3, 3));

  auto load1 = EMT::Ph3::Shunt::make("load");
  load1->setParameters(1e9, 0.0);

  auto load2 = EMT::Ph3::Shunt::make("load");
  load2->setParameters(10, 0.0);

  auto sw1 = EMT::Ph3::Switch::make("sw1");
  sw1->setParameters(switchOpenRes * Matrix::Identity(3, 3),
                     switchClosedRes * Matrix::Identity(3, 3), true);
  sw1->closeSwitch();

  auto sw2 = EMT::Ph3::Switch::make("sw2");
  sw2->setParameters(switchOpenRes * Matrix::Identity(3, 3),
                     switchClosedRes * Matrix::Identity(3, 3));
  sw2->openSwitch();

  auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<Real>::make("n3", PhaseType::ABC);
  auto n4 = SimNode<Real>::make("n4", PhaseType::ABC);

  //topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});
  r_v->connect(SimNode<Real>::List{n1, n2});

  motor->connect(SimNode<Real>::List{SimNode<Real>::GND, n2});

  sw1->connect(SimNode<Real>::List{n2, n3});
  load1->connect(SimNode<Real>::List{n3});

  sw2->connect(SimNode<Real>::List{n2, n4});
  load2->connect(SimNode<Real>::List{n4});

  auto sys = SystemTopology(
      nominalFrequency, SystemNodeList{n1, n2, n3, n4},
      SystemComponentList{vs, motor, load1, load2, sw1, sw2, r_v});

  //logging
  Logger::setLogDir("logs/" + simName_coldStart);
  auto logger = DataLogger::make(simName_coldStart);
  logger->logAttribute("mechanical_speed",
                       motor->attribute("mechanical_speed"));
  logger->logAttribute("mechanical_speed_pu",
                       motor->attribute("mechanical_speed_pu"));
  logger->logAttribute("slip", motor->attribute("slip"));
  logger->logAttribute("electrical_power",
                       motor->attribute("electrical_power"));
  logger->logAttribute("reactive_power", motor->attribute("reactive_power"));
  logger->logAttribute("electrical_torque",
                       motor->attribute("electrical_torque"));
  logger->logAttribute("mechanical_load_torque",
                       motor->attribute("mechanical_load_torque"));
  logger->logAttribute("stator_current_d",
                       motor->attribute("stator_current_d"));
  logger->logAttribute("stator_current_q",
                       motor->attribute("stator_current_q"));
  logger->logAttribute("stator_current_magnitude",
                       motor->attribute("stator_current_magnitude"));
  logger->logAttribute("stator_voltage_d",
                       motor->attribute("stator_voltage_d"));
  logger->logAttribute("stator_voltage_q",
                       motor->attribute("stator_voltage_q"));
  logger->logAttribute("stator_voltage_magnitude",
                       motor->attribute("stator_voltage_magnitude"));

  //simulation setup
  Simulation sim(simName_coldStart, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.doSystemMatrixRecomputation(true);
  sim.setTimeStep(stepTime);
  sim.setFinalTime(finalTime);

  //events
  auto sw_ev1 = SwitchEvent3Ph::make(switchTime, sw1, false);
  sim.addEvent(sw_ev1);
  auto sw_ev2 = SwitchEvent3Ph::make(switchTime, sw2, true);
  sim.addEvent(sw_ev2);

  //run sim
  sim.run();
}

int main(int argc, char *argv[]) {

  EMT_Ph3_SSN_InductionMotor_Circuits InductionMotorCircuits;
  InductionMotorCircuits.ColdStart_LoadSwitch();
}
