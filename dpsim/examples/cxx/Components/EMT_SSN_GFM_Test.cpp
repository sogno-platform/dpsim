// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "../Examples.h"

#include <cmath>

#include <DPsim.h>
#include <dpsim-models/EMT/EMT_Ph3_SSN_GFM.h>

using namespace DPsim;
using namespace CPS;

int main(int argc, char *argv[]) {
  // -------------------------------------------------------------------------
  // Simulation configuration
  // -------------------------------------------------------------------------
  const Real finalTime = 0.50;
  const Real timeStep = 100e-6;
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

  // This connection resistance is represented identically in PF and EMT.
  // It is deliberately moderate for the first closed-loop validation.
  const Real connectionResistance = 0.20;

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
  const Real reactivePowerReference = baseLoadReactivePower;

  const Real loadStepStartTime = 0.15;
  const Real loadStepEndTime = 0.25;

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

  auto linePF = SP::Ph1::PiLine::make("Connection", Logger::Level::debug);
  linePF->setParameters(connectionResistance, 0.0, 0.0);
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
  const String simNameEMT = simName + "_EMT";
  Logger::setLogDir("logs/" + simNameEMT);

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

  auto connection =
      EMT::Ph3::Resistor::make("Connection", Logger::Level::debug);
  connection->setParameters(
      Math::singlePhaseParameterToThreePhase(connectionResistance));

  auto baseLoad = EMT::Ph3::RXLoad::make("BaseLoad", Logger::Level::debug);
  baseLoad->setParameters(
      Math::singlePhasePowerToThreePhase(baseLoadActivePower),
      Math::singlePhasePowerToThreePhase(baseLoadReactivePower),
      lineToLineVoltageRms);

  auto stepLoad = EMT::Ph3::RXLoad::make("StepLoad", Logger::Level::debug);
  stepLoad->setParameters(
      Math::singlePhasePowerToThreePhase(stepLoadActivePower),
      Math::singlePhasePowerToThreePhase(stepLoadReactivePower),
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
  connection->connect({nGfmEMT, nLoadEMT});
  baseLoad->connect({nLoadEMT});
  loadSwitch->connect({nLoadEMT, nStepLoadEMT});
  stepLoad->connect({nStepLoadEMT});

  auto systemEMT = SystemTopology(
      nominalFrequency, SystemNodeList{nGfmEMT, nLoadEMT, nStepLoadEMT},
      SystemComponentList{gfm, connection, baseLoad, loadSwitch, stepLoad});

  systemEMT.initWithPowerflow(systemPF, Domain::EMT);

  // Keep the terminal operating-point metadata consistent with the initial
  // power-flow solution and the GFM reference.
  gfm->terminal(1)->setPower(
      Complex(baseLoadActivePower, baseLoadReactivePower));

  // -------------------------------------------------------------------------
  // Logging
  // -------------------------------------------------------------------------
  auto loggerEMT = DataLogger::make(simNameEMT);

  loggerEMT->logAttribute("Voltage_GFM", nGfmEMT->attribute("v"));
  loggerEMT->logAttribute("Voltage_Load", nLoadEMT->attribute("v"));
  loggerEMT->logAttribute("Voltage_StepLoad", nStepLoadEMT->attribute("v"));

  loggerEMT->logAttribute("GFM_InterfaceVoltage", gfm->attribute("v_intf"));
  loggerEMT->logAttribute("GFM_InterfaceCurrent", gfm->attribute("i_intf"));
  loggerEMT->logAttribute("GFM_ActivePower", gfm->attribute("p_inst"));
  loggerEMT->logAttribute("GFM_ReactivePower", gfm->attribute("q_inst"));
  loggerEMT->logAttribute("GFM_Omega", gfm->attribute("omega_gfm"));
  loggerEMT->logAttribute("GFM_Theta", gfm->attribute("theta_gfm"));
  loggerEMT->logAttribute("GFM_VoltageMagnitude",
                          gfm->attribute("voltage_magnitude_gfm"));
  loggerEMT->logAttribute("GFM_CapacitorVoltageD", gfm->attribute("vc_d"));
  loggerEMT->logAttribute("GFM_CapacitorVoltageQ", gfm->attribute("vc_q"));
  loggerEMT->logAttribute("GFM_GridCurrentD", gfm->attribute("i_grid_d"));
  loggerEMT->logAttribute("GFM_GridCurrentQ", gfm->attribute("i_grid_q"));
  loggerEMT->logAttribute("GFM_FilterCurrentD", gfm->attribute("if_d"));
  loggerEMT->logAttribute("GFM_FilterCurrentQ", gfm->attribute("if_q"));

  // -------------------------------------------------------------------------
  // Events and simulation
  // -------------------------------------------------------------------------
  Simulation sim(simNameEMT, Logger::Level::debug);

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
  sim.run();

  return 0;
}
