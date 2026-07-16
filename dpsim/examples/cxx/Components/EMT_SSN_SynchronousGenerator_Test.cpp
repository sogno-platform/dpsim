// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "../Examples.h"

#include <cmath>

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_SynchronousGenerator.h>

using namespace DPsim;
using namespace CPS;

int main(int argc, char *argv[]) {

  const Real finalTime = 3.0;
  const Real timeStep = 100e-6;
  const String simName = "EMT_SSN_SynchronousGenerator_GridConnected";

  // =========================================================================
  // Machine ratings from Fig. 4 of Dufour's paper
  // =========================================================================
  const Real nominalApparentPower = 202e6;
  const Real nominalVoltage = 13.8e3;
  const Real nominalFrequency = 60.0;
  const Real nominalOmega = 2.0 * PI * nominalFrequency;

  // The paper does not state the pole count in Fig. 4. For this isolated
  // component validation, one pole pair is used

  const Int polePairs = 1;
  const Real nominalMechanicalSpeed =
      nominalOmega / static_cast<Real>(polePairs);

  // Paper operating point: 0.8 pu generated active power at approximately
  // 1 pu terminal voltage. A zero-reactive-power operating point is used for
  // the first validation because the figure does not state initial Q.
  const Real generatedActivePowerPu = 0.8;
  const Real generatedReactivePowerPu = 0.0;

  const Real generatedActivePower =
      generatedActivePowerPu * nominalApparentPower;
  const Real generatedReactivePower =
      generatedReactivePowerPu * nominalApparentPower;

  // =========================================================================
  // Base quantities
  //
  // The paper lists winding values in pu. The implemented component expects:
  // - resistance in ohms
  // - inductance in henries
  // - voltage in volts
  // - torque in N m
  // - inertia in kg m^2
  //
  // For a three-phase machine:
  //
  //   Z_base = V_LL,RMS^2 / S_base
  //   L_base = Z_base / omega_base
  //   I_base = S_base / V_LL,RMS
  //
  // The orthonormal Park transform used by the component makes the dq voltage
  // base equal to V_LL,RMS and the dq current base equal to S_base/V_LL,RMS.
  // =========================================================================
  const Real impedanceBase =
      nominalVoltage * nominalVoltage / nominalApparentPower;
  const Real inductanceBase = impedanceBase / nominalOmega;

  // =========================================================================
  // Paper machine parameters in pu
  //
  // Values read from Fig. 4:
  //
  //   Rs   = 3.08e-3 pu
  //   Lls  = 0.124 pu
  //   Lmd  = 1.29 pu
  //   Lmq  = 0.388 pu
  //   Rf'  = 9.56e-4 pu
  //   Llf' = 0.1228 pu
  //   Rkd' = 1.262e-2 pu
  //   Llkd'= 1.96e-1 pu
  //   Rkq' = 2.12e-2 pu
  //   Llkq'= 1.44e-1 pu
  //   H    = 3 s
  //
  // The total winding inductances required by the component are:
  //
  //   Ld   = Lls + Lmd
  //   Lq   = Lls + Lmq
  //   Lf'  = Llf' + Lmd
  //   Lkd' = Llkd' + Lmd
  //   Lkq' = Llkq' + Lmq
  // =========================================================================
  const Real statorResistancePu = 3.08e-3;
  const Real statorLeakageInductancePu = 0.124;

  const Real mutualInductanceDPu = 1.29;
  const Real mutualInductanceQPu = 0.388;

  const Real fieldResistancePu = 9.56e-4;
  const Real fieldLeakageInductancePu = 0.1228;

  const Real damperResistanceDPu = 1.262e-2;
  const Real damperLeakageInductanceDPu = 1.96e-1;

  const Real damperResistanceQ1Pu = 2.12e-2;
  const Real damperLeakageInductanceQ1Pu = 1.44e-1;

  const Real inertiaConstant = 3.0;

  // =========================================================================
  // Converted machine parameters in SI units
  // =========================================================================
  const Real statorResistance = statorResistancePu * impedanceBase;
  const Real fieldResistance = fieldResistancePu * impedanceBase;
  const Real damperResistanceD = damperResistanceDPu * impedanceBase;
  const Real damperResistanceQ1 = damperResistanceQ1Pu * impedanceBase;

  const Real mutualInductanceD = mutualInductanceDPu * inductanceBase;
  const Real mutualInductanceQ = mutualInductanceQPu * inductanceBase;

  const Real statorInductanceD =
      (statorLeakageInductancePu + mutualInductanceDPu) * inductanceBase;

  const Real statorInductanceQ =
      (statorLeakageInductancePu + mutualInductanceQPu) * inductanceBase;

  const Real fieldInductance =
      (fieldLeakageInductancePu + mutualInductanceDPu) * inductanceBase;

  const Real damperInductanceD =
      (damperLeakageInductanceDPu + mutualInductanceDPu) * inductanceBase;

  const Real damperInductanceQ1 =
      (damperLeakageInductanceQ1Pu + mutualInductanceQPu) * inductanceBase;

  // The machine in Fig. 4 is parameterized with one q-axis damper winding.
  // The current component class has a second q-axis damper state because it
  // implements the paper's general sixth-order round-rotor form.
  //
  // To reproduce the Fig. 4 machine without changing the component API, the
  // second q-axis damper is made electrically inactive with a large leakage
  // inductance and a matching resistance. These values are numerical adapter
  // values, not values from the paper.
  const Real disabledDamperQ2Pu = 1000.0;
  const Real damperResistanceQ2 = disabledDamperQ2Pu * impedanceBase;
  const Real damperInductanceQ2 = disabledDamperQ2Pu * inductanceBase;

  // Convert H to physical rotor inertia:
  //
  //   H = (0.5 * J * omega_m^2) / S_base
  //
  // therefore
  //
  //   J = 2 * H * S_base / omega_m^2
  const Real rotorInertia = 2.0 * inertiaConstant * nominalApparentPower /
                            (nominalMechanicalSpeed * nominalMechanicalSpeed);

  // Mechanical damping is not specified in the paper. Keep it zero in the
  // first validation so that no undocumented damping changes the model.
  const Real mechanicalDamping = 0.0;

  // =========================================================================
  // Initial operating point
  //
  // The following values were obtained by solving the paper's steady-state
  // machine equations with:
  //
  //   V_terminal = 1.0 pu
  //   P_generator = 0.8 pu
  //   Q_generator = 0.0 pu
  //
  // using the same orthonormal Park transform and current-entering-machine
  // sign convention as SSN_SynchronousGenerator.
  //
  // Positive component electrical power means absorption. Therefore a
  // generator exporting 0.8 pu has P_component = -0.8 pu.
  // =========================================================================
  const Real initialElectricalAngle = 1.95868864;

  // Field voltage is referred to the stator, as required by the machine state
  // equations. Its sign follows the winding orientation in the implemented
  // inductance matrix.
  const Real fieldVoltage = -13.86626959;

  // At 0.8 pu generation and one pole pair:
  //
  //   T_mech = P_generated / omega_m
  //
  // The component's electrical torque is negative in generator operation, so
  // a positive mechanical shaft torque balances it.

  const Real mechanicalTorque = generatedActivePower / nominalMechanicalSpeed;

  // =========================================================================
  // Small load-step disturbance
  //
  // The synchronous generator remains connected to a stiff 13.8 kV source.
  // Closing the switch adds a 5% rated resistive load at the machine bus.
  // This is deliberately small for the first stability test.
  // =========================================================================

  const Real disturbancePower = 0.05 * nominalApparentPower;
  const Real disturbanceResistance =
      nominalVoltage * nominalVoltage / disturbancePower;

  const Real loadStepStartTime = 1.0;
  const Real loadStepEndTime = 2.0;

  const Real switchOpenResistance = 1e9;
  const Real switchClosedResistance = 1e-6;

  const Real gridConnectionResistance = 0.01;

  const String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto nGridPF = SimNode<Complex>::make("nGrid", PhaseType::Single);
  auto nMachinePF = SimNode<Complex>::make("nMachine", PhaseType::Single);

  auto slackPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
  slackPF->setParameters(nominalVoltage);
  slackPF->setBaseVoltage(nominalVoltage);
  slackPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto gridConnectionPF =
      SP::Ph1::PiLine::make("GridConnection", Logger::Level::debug);
  gridConnectionPF->setParameters(gridConnectionResistance, 0.0, 0.0);
  gridConnectionPF->setBaseVoltage(nominalVoltage);

  auto generatorInjectionPF =
      SP::Ph1::Load::make("SynGen", Logger::Level::debug);
  generatorInjectionPF->setParameters(-generatedActivePower,
                                      -generatedReactivePower, nominalVoltage);
  generatorInjectionPF->modifyPowerFlowBusType(PowerflowBusType::PQ);

  slackPF->connect({nGridPF});
  gridConnectionPF->connect({nGridPF, nMachinePF});
  generatorInjectionPF->connect({nMachinePF});

  auto systemPF = SystemTopology(
      nominalFrequency, SystemNodeList{nGridPF, nMachinePF},
      SystemComponentList{slackPF, gridConnectionPF, generatorInjectionPF});

  auto loggerPF = DataLogger::make(simNamePF);
  loggerPF->logAttribute("Voltage_Grid", nGridPF->attribute("v"));
  loggerPF->logAttribute("Voltage_Machine", nMachinePF->attribute("v"));

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
  // EMT simulation
  // =========================================================================
  const String simNameEMT = simName + "_EMT";
  Logger::setLogDir("logs/" + simNameEMT);

  auto nGrid = SimNode<Real>::make("nGrid", PhaseType::ABC);
  auto nMachine = SimNode<Real>::make("nMachine", PhaseType::ABC);
  auto nDisturbance = SimNode<Real>::make("nDisturbance", PhaseType::ABC);

  auto slack = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);

  auto generator = EMT::Ph3::SSN_SynchronousGenerator::make(
      "SynGen", "SynGen", Logger::Level::debug);

  generator->setParameters(
      nominalFrequency, polePairs, statorResistance, fieldResistance,
      damperResistanceD, damperResistanceQ1, damperResistanceQ2,
      statorInductanceD, statorInductanceQ, mutualInductanceD,
      mutualInductanceQ, fieldInductance, damperInductanceD, damperInductanceQ1,
      damperInductanceQ2, rotorInertia, mechanicalDamping, fieldVoltage,
      mechanicalTorque, initialElectricalAngle, true);

  generator->setNumericalLinearizationParameters(1e-6, 1e-8);

  auto gridConnection =
      EMT::Ph3::Resistor::make("GridConnection", Logger::Level::debug);
  gridConnection->setParameters(
      Math::singlePhaseParameterToThreePhase(gridConnectionResistance));

  auto loadSwitch = EMT::Ph3::Switch::make("LoadSwitch", Logger::Level::debug);
  loadSwitch->setParameters(
      Math::singlePhaseParameterToThreePhase(switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(switchClosedResistance), false);
  loadSwitch->openSwitch();

  auto disturbanceLoad =
      EMT::Ph3::Resistor::make("DisturbanceLoad", Logger::Level::debug);
  disturbanceLoad->setParameters(
      Math::singlePhaseParameterToThreePhase(disturbanceResistance));

  slack->connect({nGrid});
  gridConnection->connect({nGrid, nMachine});
  generator->connect({EMT::SimNode::GND, nMachine});

  loadSwitch->connect({nMachine, nDisturbance});
  disturbanceLoad->connect({nDisturbance, EMT::SimNode::GND});

  auto systemEMT = SystemTopology(
      nominalFrequency, SystemNodeList{nGrid, nMachine, nDisturbance},
      SystemComponentList{slack, gridConnection, generator, loadSwitch,
                          disturbanceLoad});

  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  // Preserve the initial operating-point metadata used by the component.
  generator->terminal(1)->setPower(
      Complex(-generatedActivePower, -generatedReactivePower));

  // =========================================================================
  // Logging
  // =========================================================================
  auto loggerEMT = DataLogger::make(simNameEMT);

  loggerEMT->logAttribute("Voltage_Grid", nGrid->attribute("v"));
  loggerEMT->logAttribute("Voltage_Machine", nMachine->attribute("v"));
  loggerEMT->logAttribute("Voltage_Disturbance", nDisturbance->attribute("v"));
  loggerEMT->logAttribute("SynGen_InterfaceVoltage",
                          generator->attribute("v_intf"));
  loggerEMT->logAttribute("SynGen_InterfaceCurrent",
                          generator->attribute("i_intf"));
  loggerEMT->logAttribute("SynGen_ElectricalPower",
                          generator->attribute("electrical_power"));
  loggerEMT->logAttribute("SynGen_ElectricalTorque",
                          generator->attribute("electrical_torque"));
  loggerEMT->logAttribute("SynGen_MechanicalSpeed",
                          generator->attribute("mechanical_speed"));
  loggerEMT->logAttribute("SynGen_ElectricalAngle",
                          generator->attribute("electrical_angle"));
  loggerEMT->logAttribute("SynGen_StatorVoltageD",
                          generator->attribute("stator_voltage_d"));
  loggerEMT->logAttribute("SynGen_StatorVoltageQ",
                          generator->attribute("stator_voltage_q"));
  loggerEMT->logAttribute("SynGen_StatorCurrentD",
                          generator->attribute("stator_current_d"));
  loggerEMT->logAttribute("SynGen_StatorCurrentQ",
                          generator->attribute("stator_current_q"));
  loggerEMT->logAttribute("SynGen_FieldCurrent",
                          generator->attribute("field_current"));
  loggerEMT->logAttribute("DisturbanceCurrent",
                          disturbanceLoad->attribute("i_intf"));

  // =========================================================================
  // Simulation
  // =========================================================================
  Simulation sim(simNameEMT, Logger::Level::debug);

  sim.addEvent(SwitchEvent3Ph::make(loadStepStartTime, loadSwitch, true));
  sim.addEvent(SwitchEvent3Ph::make(loadStepEndTime, loadSwitch, false));

  sim.setSystem(systemEMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);

  // The Park transform and speed term make the machine admittance variable.
  // The system matrix must therefore be rebuilt and factorized every step.
  sim.doSystemMatrixRecomputation(true);

  sim.doInitFromNodesAndTerminals(true);
  sim.addLogger(loggerEMT);
  sim.run();

  return 0;
}
