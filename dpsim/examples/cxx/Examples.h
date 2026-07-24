/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <DPsim.h>

namespace CPS {
namespace CIM {
namespace Examples {

namespace Components {
namespace SynchronousGeneratorKundur {
// P. Kundur, "Power System Stability and Control", Example 3.2, pp. 102
// and Example 3.5, pp. 134f.
struct MachineParameters {
  // Thermal generating unit, 3600r/min, 2-pole
  Real nomPower = 555e6;
  Real nomVoltage = 24e3; // Phase-to-Phase RMS
  Real nomFreq = 60;
  Real nomFieldCurr = 1300;
  Int poleNum = 2;
  Real H = 3.7;

  // Define machine parameters in per unit
  // Fundamental parameters
  Real Rs = 0.003;
  Real Ll = 0.15;
  Real Lmd = 1.6599;
  Real Lmq = 1.61;
  Real Rfd = 0.0006;
  Real Llfd = 0.1648;
  Real Rkd = 0.0284;
  Real Llkd = 0.1713;
  Real Rkq1 = 0.0062;
  Real Llkq1 = 0.7252;
  Real Rkq2 = 0.0237;
  Real Llkq2 = 0.125;

  // Operational parameters
  Real Td0_t = 8.0669;
  Real Td0_s = 0.0300;
  Real Td_t = 1.3368;
  Real Td_s = 0.0230;
  Real Ld_t = 0.2999;
  Real Ld_s = 0.2299;
  Real Tq0_t = 0.9991;
  Real Tq0_s = 0.0700;
  Real Lq_t = 0.6500;
  Real Lq_s = 0.2500;
  Real Ld = 1.8099;
  Real Lq = 1.7600;
};
} // namespace SynchronousGeneratorKundur

namespace GovernorKundur {
struct Parameters {
  // Turbine model parameters (tandem compound single reheat steam turbine, fossil-fuelled)
  // from P. Kundur, "Power System Stability and Control", 1994, p. 427
  Real Ta_t = 0.3; // T_CH
  Real Fa = 0.3;   // F_HP
  Real Tb = 7.0;   // T_RH
  Real Fb = 0.3;   // F_IP
  Real Tc = 0.5;   // T_CO
  Real Fc = 0.4;   // F_LP

  // Governor parameters (mechanical-hydraulic control)
  // from P. Kundur, "Power System Stability and Control", 1994, p. 437
  Real Kg = 20; // 5% droop
  Real Tsr = 0.1;
  Real Tsm = 0.3;
};
} // namespace GovernorKundur

namespace ExcitationSystemEremia {
struct Parameters {
  // Excitation system parameters (IEEE Type DC1A)
  // from M. Eremia, "Handbook of Electrical Power System Dynamics", 2013, p.96 and 106
  // voltage-regulator
  Real Ka = 46;
  Real Ta = 0.06;
  // exciter
  Real Ke = -0.0435;
  Real Te = 0.46;
  // stabilizing feedback
  Real Kf = 0.1;
  Real Tf = 1;
  // voltage transducer
  Real Tr = 0.02;
};
} // namespace ExcitationSystemEremia

namespace TurbineGovernor {
struct TurbineGovernorPSAT1 {
  // Turbine Governor type 1
  // Taken from from PSAT - example d_014_pss_l14

  // Reference speed (p.u.)
  Real OmegaRef = 1.0;
  // Pilot valve droop (p.u.)
  Real R = 0.02;
  // Maximum Torque (p.u.)
  Real Tmax = 1.2;
  // Minimim Torque (p.u.)
  Real Tmin = 0.3;
  // Governor time constant (s)
  Real Ts = 0.1;
  // Servo time constant (s)
  Real Tc = 0.45;
  // Transient gain time constant (s)
  Real T3 = 0.0;
  // Power fraction time constant (s)
  Real T4 = 12.0;
  // Reheat time constant (s)
  Real T5 = 50.0;
};

struct TurbineGovernorPSAT2 {
  // Turbine Governor type 1
  // Taken from PSAT - example d_anderson_farmer

  // Reference speed (p.u.)
  Real OmegaRef = 1.0;
  // Pilot valve droop (p.u.)
  Real R = 0.04;
  // Maximum Torque (p.u.)
  Real Tmax = 100;
  // Minimim Torque (p.u.)
  Real Tmin = 0.0;
  // Governor time constant (s)
  Real Ts = 20;
  // Servo time constant (s)
  Real Tc = 0.2;
  // Transient gain time constant (s)
  Real T3 = 0.2;
  // Power fraction time constant (s)
  Real T4 = 0.2;
  // Reheat time constant (s)
  Real T5 = 0.2;
};
} // namespace TurbineGovernor

namespace PSS1A {
struct Parameters {
  // PSS1A (IEEE type 1A) — typical Kundur test-case values
  // Ref.: Milano - Power system modelling and scripting, p. 371
  Real Kp = 0.0;
  Real Kv = 0.0;
  Real Kw = 20.0;
  Real T1 = 0.14;
  Real T2 = 0.04;
  Real T3 = 0.14;
  Real T4 = 0.04;
  Real Vs_max = 0.1;
  Real Vs_min = -0.1;
  Real Tw = 10.0;
};
} // namespace PSS1A

namespace SteamGovernor {
struct Parameters {
  Real R = 0.04;
  Real T1 = 0.0;
  Real T2 = 0.2;
  Real T3 = 0.1;
  Real dPmax = 50;
  Real dPmin = -50;
  Real Pmax = 1.0;
  Real Pmin = 0.0;
  Real OmRef = 1.0;
  Real Kbc = 0.0;
};
} // namespace SteamGovernor

namespace SteamTurbine {
struct Parameters {
  Real Fhp = 0.3;
  Real Fip = 0.3;
  Real Flp = 0.4;
  Real Tch = 0.1;
  Real Trh = 4.0;
  Real Tco = 0.3;
};
} // namespace SteamTurbine

namespace HydroGovernor {
struct Parameters {
  Real R = 0.05;
  Real T1 = 1.0;
  Real T2 = 0.2;
  Real T3 = 0.5;
  Real Pmax = 1.0;
  Real Pmin = 0.0;
  Real OmRef = 1.0;
};
} // namespace HydroGovernor

namespace HydroTurbine {
struct Parameters {
  Real Tw = 1.0;
};
} // namespace HydroTurbine

namespace GFM {
struct Yazdani {
  // Parameters for the GFM-VSI model from
  // "Yazdani, A., & Iravani, R. (2010). Voltage-sourced converters in power systems: modeling, control, and applications. John Wiley & Sons."
  // Chapter 9.3

  // Initial state values of VSI system matrix
  Real thetaPLLInit = 0;
  Real phiPLLInit = 0;
  Real phi_dInit = 0;
  Real phi_qInit = 0;
  Real gamma_dInit = 0;
  Real gamma_qInit = 0;

  // VSI generated values
  // choose value * sqrt (3.0 / 2.0) to have phase peak at value
  // choose value to have RMS value at PCC
  Real Vdref = 400 * sqrt(3.0 / 2.0);
  Real Vqref = 0;
  Real systemFrequency = 60;
  Real OmegaNull = 2 * M_PI * systemFrequency; //System circular frequency

  // VSI filter parameters
  Real Lf = 100e-6;
  Real Cf = 2.5e-3;
  Real Rf = 2.07e-3;
  Real tau = 0.5e-3;
  Real Rc = 10e-6; //connecting resistor to external network

  // VSI controller parameters
  Real scaling_P = 1;
  Real scaling_I = 1;

  Real KpVoltageCtrl = 1.6725 * scaling_P;
  Real KiVoltageCtrl = 374.64 * scaling_I;
  Real KpCurrCtrl = 0.2 * scaling_P;
  Real KiCurrCtrl = 4.14 * scaling_I;

  // PLL controller parameters
  // OmegaCutoff is the cutoff-frequency of the PLL filter
  // in case of VCO-mode use KpPLL=0, KiPLL=0 and OmegaCutoff = OmegaNull to work as VCO
  Real KpPLL = 0;
  Real KiPLL = 0;
  Real OmegaCutoff = OmegaNull;

  // LINES AND LOAD PARAMETERS

  // Line parameters (R/X = 1)
  Real length = 5;
  Real lineResistance = 0.5 * length;
  Real lineInductance = 0.5 / 314 * length;
  Real lineCapacitance = 50e-6 / 314 * length;

  //Load Parameters
  Real Res1 = 83e-3;
  Real Ind1 = 137e-6;
  Real Res2 = 50e-3;
  Real Ind2 = 68e-6;
  Real Cap2 = 13.55e-3;
};

struct Derived {
  // System quantities
  Real lineToLineVoltageRms =
      20000.0 * sqrt(3.0 / 2.0); // RMS line-to-line voltage at PCC
  Real systemFrequency = 60.0;
  Real OmegaNull = 2.0 * M_PI * systemFrequency;

  // Power-invariant dq reference:
  // For a balanced system, Vd equals line-to-line RMS voltage.
  Real Vdref = lineToLineVoltageRms;
  Real Vqref = 0.0;

  // Filter
  Real Lf = 100e-6;
  Real Cf = 2.5e-3;
  Real Rf = 2.07e-3;
  Real Rc = 1e-5;
  Real tau = 0.5e-3;

  // Controller design specifications
  Real currentCtrlBandwidth = 2000.0;       // rad/s
  Real voltageCtrlNaturalFrequency = 387.1; // rad/s
  Real voltageCtrlDamping = 0.864;

  // Calculated current-controller gains
  Real KpCurrCtrl = Lf * currentCtrlBandwidth;
  Real KiCurrCtrl = Rf * currentCtrlBandwidth;

  // Calculated voltage-controller gains
  Real KpVoltageCtrl =
      2.0 * voltageCtrlDamping * voltageCtrlNaturalFrequency * Cf;

  Real KiVoltageCtrl =
      voltageCtrlNaturalFrequency * voltageCtrlNaturalFrequency * Cf;

  // VCO mode --> for PLL change values
  Real KpPLL = 0.0;
  Real KiPLL = 0.0;
  Real OmegaCutoff = OmegaNull;

  // Initial states
  Real thetaPLLInit = 0.0;
  Real phiPLLInit = 0.0;
  Real phi_dInit = 0.0;
  Real phi_qInit = 0.0;
  Real gamma_dInit = 0.0;
  Real gamma_qInit = 0.0;

  // Network
  Real length = 5.0;
  Real lineResistance = 0.5 * length;
  Real lineInductance = 0.5 / 314.0 * length;
  Real lineCapacitance = 50e-6 / 314.0 * length;

  // Loads
  Real Res1 = 83e-3;
  Real Ind1 = 137e-6;
  Real Res2 = 50e-3;
  Real Ind2 = 68e-6;
  Real Cap2 = 13.55e-3;
};

struct Ieee9SsnGridForming {
  // Parameters for the EMT::Ph3::SSN_GFM grid-forming inverter at BUS2 of the
  // IEEE 9-bus mixed-inverter example. Control structure (VSG loop + cascaded
  // voltage/current control + active damping) follows
  //   X. Gao, D. Zhou, A. Anvari-Moghaddam, F. Blaabjerg, "Stability Analysis
  //   of Grid-Following and Grid-Forming Converters Based on State-Space
  //   Model", IPEC-Himeji 2022 (ECCE Asia), pp. 422-428.
  // The LC filter is the PR-#570 reference design (220 V / 12 kVA / 50 Hz)
  // rebased to the BUS2 base, per-unit preserved. The cascaded-loop gains are
  // computed from design bandwidths, not hand-set.

  // BUS2 base
  Real ratedVoltage = 18e3;    // V line-to-line RMS
  Real ratedPower = 100e6;     // VA
  Real systemFrequency = 60.0; // Hz
  Real OmegaNull = 2.0 * M_PI * systemFrequency;

  // LC filter and coupling resistance (BUS2 base)
  Real Lf = 6.694215e-4; // H
  Real Cf = 6.224280e-5; // F
  Real Rf = 1.338843e-2; // Ohm
  Real Rc = 1.338843e-2; // Ohm

  // Inner current loop: PI cancels the filter pole, so the closed loop is a
  // first-order lag of bandwidth currentCtrlBandwidth (Kp = Lf*w, Ki = Rf*w).
  Real currentCtrlBandwidth = 1683.03;        // rad/s
  Real KpCurrent = Lf * currentCtrlBandwidth; // = 1.126636
  Real KiCurrent = Rf * currentCtrlBandwidth; // = 22.53273

  // Outer voltage loop: second-order shaping on Cf. The natural frequency is
  // kept low for stiff-grid stability (Kp = 2*zeta*wn*Cf, Ki = wn^2*Cf).
  Real voltageCtrlNaturalFrequency = 65.865; // rad/s
  Real voltageCtrlDamping = 1.0 / sqrt(2.0);
  Real KpVoltage =
      2.0 * voltageCtrlDamping * voltageCtrlNaturalFrequency * Cf; // = 5.8e-3
  Real KiVoltage =
      voltageCtrlNaturalFrequency * voltageCtrlNaturalFrequency * Cf; // = 0.27

  // VSG swing: virtual inertia J and damping D. D is raised well above the
  // islanded value for stiff-grid stability.
  Real virtualInertia = 1157.407407;
  Real dampingCoefficient = 3.0e6;

  // Integral excitation (superseded by the proportional Q-V droop below, but
  // still passed to setParameters for the islanded fallback).
  Real voltageDroopGain = 1.0 / 15.0;
  Real reactiveIntegralGain = 1.700559e-3;

  // Active damping off (the converter delay makes its sign uncertain here),
  // power-measurement filter cutoff, and switching/delay bandwidth.
  Real activeDampingGain = 0.0;
  Real powerFilterCutoff = 100.0; // rad/s
  Real switchingFrequency = 20e3; // Hz
  Real delayBandwidth = switchingFrequency / 1.5;

  // Grid-connected control extensions (opt-in on the model). Feedforward is
  // turned off and the integral excitation replaced by a proportional Q-V
  // droop, both for stiff-grid stability.
  Real gridCurrentFeedforward = 0.0; // 1 = full islanded feedforward
  Real reactivePowerDroop = 1.0e-5;  // Dq [V/var]
  Real reactiveDroopCutoff = 100.0;  // rad/s
};
} // namespace GFM

namespace GFL {
struct Ieee9AvVsi {
  // Parameters for the EMT::Ph3::AvVoltSourceInverterStateSpace grid-following
  // inverter at BUS3. The 400 V / 10 kVA reference design is rebased to the
  // BUS3 base (13.8 kV, 100 MVA, 60 Hz): PLL and power-loop gains scale by the
  // inverse voltage ratio, current-loop gains by the impedance ratio, so the
  // per-unit control is preserved.
  Real ratedVoltage = 13.8e3; // V line-to-line RMS
  Real ratedPower = 100e6;    // VA
  Real systemFrequency = 60.0;
  Real OmegaNull = 2.0 * M_PI * systemFrequency;

  // LC filter and coupling resistance (BUS3 base)
  Real Lf = 1.98375e-4; // H
  Real Cf = 7.00133e-5; // F
  Real Rf = 2.38050e-2; // Ohm
  Real Rc = 2.38050e-2; // Ohm

  // PLL and power loop (rebased by 1/voltageRatio, voltageRatio = 13.8kV/400V)
  Real KpPLL = 7.246377e-3;
  Real KiPLL = 5.797101e-3;
  Real KpPowerCtrl = 1.449275e-3;
  Real KiPowerCtrl = 5.797101e-3;

  // Current loop (rebased by the impedance ratio)
  Real KpCurrCtrl = 2.975625e-2;
  Real KiCurrCtrl = 1.190250e-1;
};
} // namespace GFL
} // namespace Components

namespace Grids {
namespace CIGREHVEuropean {
struct LineParameters {
  // 220 kV
  Real Vnom = 220e3;
  Real nomFreq = 50;
  Real nomOmega = nomFreq * 2 * PI;
  //PiLine parameters (given in [p.u/km] with Z_base= 529ohm)
  Real lineResistancePerKm = 1.35e-4 * 529;
  Real lineReactancePerKm = 8.22e-4 * 529;
  Real lineSusceptancePerKm = 1.38e-3 / 529;
  Real lineConductancePerKm = 0;
};
} // namespace CIGREHVEuropean
namespace CIGREHVAmerican {
struct LineParameters {
  // 230 kV
  Real Vnom = 230e3;
  Real nomFreq = 60;
  Real nomOmega = nomFreq * 2 * PI;
  //PiLine parameters (given in [p.u/km] with Z_base= 529ohm)
  Real lineResistancePerKm = 1.27e-4 * 529;
  Real lineReactancePerKm = 9.05e-4 * 529;
  Real lineSusceptancePerKm = 1.81e-3 / 529;
  Real lineConductancePerKm = 0;
};
} // namespace CIGREHVAmerican

namespace KundurExample1 {
// P. Kundur, "Power System Stability and Control", Example 13.2, pp. 864-869.
struct Network {
  Real nomVoltage = 400e3;
};

struct Gen {
  Real nomPower = 2220e6;
  Real nomVoltage = 400e3;
  Real H = 3.5;
  Real XpdPU = 0.3;
  Real RsPU = 0;
  Real D = 1.0;
};
struct Line1 {
  // Vnom = 400kV
  Real lineResistance = 0.0721;
  Real lineReactance = 36.0360;
  Real lineSusceptance = 0;
  Real lineConductance = 0;
};

struct Transf1 {
  Real nomVoltageHV = 400e3;
  Real nomVoltageMV = 400e3;
  Real transformerResistance = 0;      // referred to HV side
  Real transformerReactance = 10.8108; // referred to HV side
};
} // namespace KundurExample1

namespace SMIB {
struct ScenarioConfig {
  //-----------Network-----------//
  Real Vnom = 230e3;
  Real nomFreq = 60;
  Real nomOmega = nomFreq * 2 * PI;
  //-----------Generator-----------//
  Real nomPower = 500e6;
  Real nomPhPhVoltRMS = 22e3;
  Real H = 5;
  Real Xpd = 0.31;
  Real Rs = 0.003 * 0;
  Real D = 1.5;
  // Initialization parameters
  Real initMechPower = 300e6;
  Real initActivePower = 300e6;
  Real setPointVoltage = nomPhPhVoltRMS + 0.05 * nomPhPhVoltRMS;
  //-----------Transformer-----------//
  Real t_ratio = Vnom / nomPhPhVoltRMS;
  //-----------Transmission Line-----------//
  // CIGREHVAmerican (230 kV)
  Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
  Real lineLeng = 100;
  Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLeng;
  Real lineInductance = lineCIGREHV.lineReactancePerKm / nomOmega * lineLeng;
  Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm / nomOmega * lineLeng;
  Real lineConductance = lineCIGREHV.lineConductancePerKm * lineLeng;
};

struct ScenarioConfig2 {
  //Scenario used to validate reduced order SG VBR models against PSAT (in SP domain)

  // General grid parameters
  Real VnomMV = 24e3;
  Real VnomHV = 230e3;
  Real nomFreq = 60;
  Real ratio = VnomMV / VnomHV;
  Real nomOmega = nomFreq * 2 * PI;

  // Generator parameters
  Real setPointActivePower = 300e6;
  Real setPointVoltage = 1.05 * VnomMV;

  // CIGREHVAmerican (230 kV)
  Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
  Real lineLength = 100;
  Real lineResistance =
      lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio, 2);
  Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength *
                        std::pow(ratio, 2) / nomOmega;
  Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength /
                         std::pow(ratio, 2) / nomOmega;
  Real lineConductance = 1e-15;

  // In PSAT SwitchClosed is equal to 1e-3 p.u.
  Real SwitchClosed = 0.001 * (24 * 24 / 555);
  Real SwitchOpen = 1e6;
};

struct ScenarioConfig3 {
  //Scenario used to validate reduced order SG VBR models in the DP and EMT domain against the SP domain

  // General grid parameters
  Real VnomMV = 24e3;
  Real nomFreq = 60;
  Real nomOmega = nomFreq * 2 * PI;

  //-----------Generator-----------//
  Real setPointActivePower = 300e6;
  Real mechPower = 300e6;
  Real initActivePower = 300e6;
  Real initReactivePower = 0;
  Real initVoltAngle = -PI / 2;
  Complex initComplexElectricalPower =
      Complex(initActivePower, initReactivePower);
  Complex initTerminalVolt =
      VnomMV * Complex(cos(initVoltAngle), sin(initVoltAngle));

  //
  Real SwitchClosed = 0.1;
  Real SwitchOpen = 1e6;
};

namespace ReducedOrderSynchronGenerator {
namespace Scenario4 {
//Scenario used to compare DP against SP accuracy in Martin's thesis

struct Config {
  // default configuration of scenario
  // adjustable using applyCommandLineArgsOptions
  String sgType = "4";
  Real startTimeFault = 30.0;
  Real endTimeFault = 30.1;
};

struct GridParams {
  // General grid parameters
  Real VnomMV = 24e3;
  Real VnomHV = 230e3;
  Real nomFreq = 60;
  Real ratio = VnomMV / VnomHV;
  Real nomOmega = nomFreq * 2 * PI;

  // Generator parameters
  Real setPointActivePower = 300e6;
  Real setPointVoltage = 1.05 * VnomMV;

  // CIGREHVAmerican (230 kV)
  Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
  Real lineLength = 100;
  Real lineResistance =
      lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio, 2);
  Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength *
                        std::pow(ratio, 2) / nomOmega;
  Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength /
                         std::pow(ratio, 2) / nomOmega;
  Real lineConductance = 8e-2;

  // In PSAT SwitchClosed is equal to 1e-3 p.u.
  Real SwitchClosed = 0.1;
  Real SwitchOpen = 1e6;
};
} // namespace Scenario4

namespace Scenario5 {
// SMIB scenario with RX trafo and load step as event
struct Config {
  // default configuration of scenario
  // adjustable using applyCommandLineArgsOptions
  String sgType = "4";
  Real startTimeFault = 1.0;
  Real endTimeFault = 1.1;
};

struct GridParams {

  // General grid parameters
  Real VnomMV = 24e3;
  Real VnomHV = 230e3;
  Real nomFreq = 60;
  Real ratio = VnomMV / VnomHV;
  Real nomOmega = nomFreq * 2 * PI;

  // Generator parameters
  Real setPointActivePower = 300e6;
  Real setPointVoltage = 1.05 * VnomMV;

  // CIGREHVAmerican (230 kV)
  Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
  Real lineLength = 100;
  Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength;
  Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength / nomOmega;
  Real lineCapacitance =
      lineCIGREHV.lineSusceptancePerKm * lineLength / nomOmega;
  Real lineConductance = 1.0491e-05; // Psnub 0.1% of 555MW

  // Switch for load step
  Real SwitchClosed = 529;      // 100 MW load step
  Real SwitchOpen = 9.1840e+07; // corresponds to 1e6 Ohms at MV level of 24kV
};

struct Transf1 {
  Real nomVoltageHV = 230e3;
  Real nomVoltageMV = 24e3;
  Real transformerResistance = 0;     // referred to HV side
  Real transformerReactance = 5.2900; // referred to HV side
  Real transformerNominalPower = 555e6;
};
} // namespace Scenario5

namespace Scenario6 {
// SMIB scenario with ideal trafo and load step as event

struct Config {
  // default configuration of scenario
  // adjustable using applyCommandLineArgsOptions
  String sgType = "4";
  Real loadStepEventTime = 10.0;

  // parameters of iterative model
  Real maxIter = 25;
  Real tolerance = 1e-10;
};

struct GridParams {
  // General grid parameters
  Real VnomMV = 24e3;
  Real VnomHV = 230e3;
  Real nomFreq = 60;
  Real ratio = VnomMV / VnomHV;
  Real nomOmega = nomFreq * 2 * PI;

  // Generator parameters
  Real setPointActivePower = 300e6;
  Real setPointVoltage = 1.05 * VnomMV;

  // CIGREHVAmerican (230 kV)
  Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
  Real lineLength = 100;
  Real lineResistance =
      lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio, 2);
  Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength *
                        std::pow(ratio, 2) / nomOmega;
  Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength /
                         std::pow(ratio, 2) / nomOmega;
  Real lineConductance = 0.0048; // Psnub 0.5% of 555MW

  // Load step
  Real loadStepActivePower = 100e6;
};
} // namespace Scenario6

} // namespace ReducedOrderSynchronGenerator
} // namespace SMIB

namespace ThreeBus {
struct ScenarioConfig {

  //-----------Network-----------//
  Real Vnom = 230e3;
  Real nomFreq = 60;
  Real nomOmega = nomFreq * 2 * PI;

  //-----------Generator 1 (bus1)-----------//
  Real nomPower_G1 = 300e6;
  Real nomPhPhVoltRMS_G1 = 25e3;
  Real nomFreq_G1 = 60;
  Real H_G1 = 6;
  Real Xpd_G1 = 0.3;      //in p.u
  Real Rs_G1 = 0.003 * 0; //in p.u
  Real D_G1 = 1.5;        //in p.u
  // Initialization parameters
  Real initActivePower_G1 = 270e6;
  Real initMechPower_G1 = 270e6;
  Real setPointVoltage_G1 = nomPhPhVoltRMS_G1 + 0.05 * nomPhPhVoltRMS_G1;

  //-----------Generator 2 (bus2)-----------//
  Real nomPower_G2 = 50e6;
  Real nomPhPhVoltRMS_G2 = 13.8e3;
  Real nomFreq_G2 = 60;
  Real H_G2 = 2;
  Real Xpd_G2 = 0.1;      //in p.u
  Real Rs_G2 = 0.003 * 0; //in p.u
  Real D_G2 = 1.5;        //in p.u
  // Initialization parameters
  Real initActivePower_G2 = 45e6;
  Real initMechPower_G2 = 45e6;
  Real setPointVoltage_G2 = nomPhPhVoltRMS_G2 - 0.05 * nomPhPhVoltRMS_G2;

  //-----------Transformers-----------//
  Real t1_ratio = Vnom / nomPhPhVoltRMS_G1;
  Real t2_ratio = Vnom / nomPhPhVoltRMS_G2;

  //-----------Load (bus3)-----------
  Real activePower_L = 310e6;
  Real reactivePower_L = 150e6;

  // -----------Transmission Lines-----------//
  // CIGREHVAmerican (230 kV)
  Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
  //line 1-2 (180km)
  Real lineResistance12 = lineCIGREHV.lineResistancePerKm * 180;
  Real lineInductance12 = lineCIGREHV.lineReactancePerKm / nomOmega * 180;
  Real lineCapacitance12 = lineCIGREHV.lineSusceptancePerKm / nomOmega * 180;
  Real lineConductance12 = lineCIGREHV.lineConductancePerKm * 180;
  //line 1-3 (150km)
  Real lineResistance13 = lineCIGREHV.lineResistancePerKm * 150;
  Real lineInductance13 = lineCIGREHV.lineReactancePerKm / nomOmega * 150;
  Real lineCapacitance13 = lineCIGREHV.lineSusceptancePerKm / nomOmega * 150;
  Real lineConductance13 = lineCIGREHV.lineConductancePerKm * 150;
  //line 2-3 (80km)
  Real lineResistance23 = lineCIGREHV.lineResistancePerKm * 80;
  Real lineInductance23 = lineCIGREHV.lineReactancePerKm / nomOmega * 80;
  Real lineCapacitance23 = lineCIGREHV.lineSusceptancePerKm / nomOmega * 80;
  Real lineConductance23 = lineCIGREHV.lineConductancePerKm * 80;
};
} // namespace ThreeBus

namespace SGIB {

struct ScenarioConfig {
  Real systemFrequency = 50;
  Real systemNominalVoltage = 20e3;

  // Line parameters (R/X = 1)
  Real length = 5;
  Real lineResistance = 0.5 * length;
  Real lineInductance = 0.5 / 314 * length;
  Real lineCapacitance = 50e-6 / 314 * length;

  // PV controller parameters
  Real scalingKp = 1;
  Real scalingKi = 0.1;

  Real KpPLL = 0.25 * scalingKp;
  Real KiPLL = 2 * scalingKi;
  Real KpPowerCtrl = 0.001 * scalingKp;
  Real KiPowerCtrl = 0.08 * scalingKi;
  Real KpCurrCtrl = 0.3 * scalingKp;
  Real KiCurrCtrl = 10 * scalingKi;
  Real OmegaCutoff = 2 * PI * systemFrequency;

  // Initial state values
  Real thetaPLLInit = 0; // only for debug
  Real phiPLLInit = 0;   // only for debug
  Real phi_dInit = 0;
  Real phi_qInit = 0;
  Real gamma_dInit = 0;
  Real gamma_qInit = 0;

  // Nominal generated power values of PV
  Real pvNominalVoltage = 1500.;
  Real pvNominalActivePower = 100e3;
  Real pvNominalReactivePower = 50e3;

  // PV filter parameters
  Real Lf = 0.002;
  Real Cf = 789.3e-6;
  Real Rf = 0.1;
  Real Rc = 0.1;

  // PV connection transformer parameters
  Real transformerNominalPower = 5e6;
  Real transformerInductance = 0.928e-3;

  // Further parameters
  Real systemOmega = 2 * PI * systemFrequency;
};
} // namespace SGIB

namespace CIGREMV {

struct ScenarioConfig {
  Real systemFrequency = 50;
  Real systemNominalVoltage = 20e3;
  Real penetrationLevel = 1;
  Real totalLoad =
      4319.1e3; // calculated total load in CIGRE MV left feeder (see CIM data)

  // parameters of one PV unit
  Real pvUnitNominalVoltage = 1500.;
  Real pvUnitNominalPower = 50e3;
  Real pvUnitPowerFactor = 1;

  // calculate PV units per plant to reach penetration level
  Int numberPVUnits = Int(totalLoad * penetrationLevel / pvUnitNominalPower);
  Int numberPVPlants = 9;
  Int numberPVUnitsPerPlant = numberPVUnits / numberPVPlants;

  // PV controller parameters
  Real scalingKp = 1;
  Real scalingKi = 0.001;

  Real KpPLL = 0.25 * scalingKp;
  Real KiPLL = 2 * scalingKi;
  Real KpPowerCtrl = 0.001 * scalingKp;
  Real KiPowerCtrl = 0.08 * scalingKi;
  Real KpCurrCtrl = 0.3 * scalingKp;
  Real KiCurrCtrl = 10 * scalingKi;
  Real OmegaCutoff = 2 * PI * systemFrequency;

  // PV filter parameters
  Real Lf = 0.002;
  Real Cf = 789.3e-6;
  Real Rf = 0.1;
  Real Rc = 0.1;

  // PV connection transformer parameters
  Real transformerNominalPower = 5e6;
  Real transformerInductance = 0.928e-3;

  // Further parameters
  Real systemOmega = 2 * PI * systemFrequency;

  // Initial state values (use known values with scaled control params)
  Real thetaPLLInit = 314.168313 - systemOmega;
  Real phiPLLInit = 8e-06;
  Real pInit = 450000.716605;
  Real qInit = -0.577218;
  Real phi_dInit = 3854197405 * scalingKi;
  Real phi_qInit = -3737 * scalingKi;
  Real gamma_dInit = 128892668 * scalingKi;
  Real gamma_qInit = 23068682 * scalingKi;
};

void addInvertersToCIGREMV(SystemTopology &system,
                           CIGREMV::ScenarioConfig scenario, Domain domain) {
  Real pvActivePower =
      scenario.pvUnitNominalPower * scenario.numberPVUnitsPerPlant;
  Real pvReactivePower =
      sqrt(std::pow(pvActivePower / scenario.pvUnitPowerFactor, 2) -
           std::pow(pvActivePower, 2));

  // add PVs to network topology
  for (Int n = 3; n <= 11; ++n) {
    // TODO: cast to BaseAverageVoltageSourceInverter and move set functions out of case distinction
    if (domain == Domain::SP) {
      SimNode<Complex>::Ptr connectionNode =
          system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
      auto pv = SP::Ph1::AvVoltageSourceInverterDQ::make(
          "pv_" + connectionNode->name(), "pv_" + connectionNode->name(),
          Logger::Level::debug, true);
      pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage,
                        pvActivePower, pvReactivePower);
      pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL,
                                  scenario.KpPowerCtrl, scenario.KiPowerCtrl,
                                  scenario.KpCurrCtrl, scenario.KiCurrCtrl,
                                  scenario.OmegaCutoff);
      pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf,
                              scenario.Rc);
      pv->setTransformerParameters(
          scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage,
          scenario.transformerNominalPower,
          scenario.systemNominalVoltage / scenario.pvUnitNominalVoltage, 0, 0,
          scenario.transformerInductance);
      pv->setInitialStateValues(scenario.pInit, scenario.qInit,
                                scenario.phi_dInit, scenario.phi_qInit,
                                scenario.gamma_dInit, scenario.gamma_qInit);
      system.addComponent(pv);
      system.connectComponentToNodes<Complex>(pv, {connectionNode});
    } else if (domain == Domain::DP) {
      SimNode<Complex>::Ptr connectionNode =
          system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
      auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make(
          "pv_" + connectionNode->name(), "pv_" + connectionNode->name(),
          Logger::Level::debug, true);
      pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage,
                        pvActivePower, pvReactivePower);
      pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL,
                                  scenario.KpPowerCtrl, scenario.KiPowerCtrl,
                                  scenario.KpCurrCtrl, scenario.KiCurrCtrl,
                                  scenario.OmegaCutoff);
      pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf,
                              scenario.Rc);
      pv->setTransformerParameters(
          scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage,
          scenario.transformerNominalPower,
          scenario.systemNominalVoltage / scenario.pvUnitNominalVoltage, 0, 0,
          scenario.transformerInductance);
      pv->setInitialStateValues(scenario.pInit, scenario.qInit,
                                scenario.phi_dInit, scenario.phi_qInit,
                                scenario.gamma_dInit, scenario.gamma_qInit);
      system.addComponent(pv);
      system.connectComponentToNodes<Complex>(pv, {connectionNode});
    } else if (domain == Domain::EMT) {
      SimNode<Real>::Ptr connectionNode =
          system.node<CPS::SimNode<Real>>("N" + std::to_string(n));
      auto pv = EMT::Ph3::AvVoltageSourceInverterDQ::make(
          "pv_" + connectionNode->name(), "pv_" + connectionNode->name(),
          Logger::Level::debug, true);
      pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage,
                        pvActivePower, pvReactivePower);
      pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL,
                                  scenario.KpPowerCtrl, scenario.KiPowerCtrl,
                                  scenario.KpCurrCtrl, scenario.KiCurrCtrl,
                                  scenario.OmegaCutoff);
      pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf,
                              scenario.Rc);
      pv->setTransformerParameters(
          scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage,
          scenario.transformerNominalPower,
          scenario.systemNominalVoltage / scenario.pvUnitNominalVoltage, 0, 0,
          scenario.transformerInductance, scenario.systemOmega);
      pv->setInitialStateValues(scenario.pInit, scenario.qInit,
                                scenario.phi_dInit, scenario.phi_qInit,
                                scenario.gamma_dInit, scenario.gamma_qInit);
      system.addComponent(pv);
      system.connectComponentToNodes<Real>(pv, {connectionNode});
    }
  }
}

void logPVAttributes(DPsim::DataLogger::Ptr logger,
                     CPS::TopologicalPowerComp::Ptr pv) {

  // power controller
  std::vector<String> inputNames = {pv->name() + "_powerctrl_input_pref",
                                    pv->name() + "_powerctrl_input_qref",
                                    pv->name() + "_powerctrl_input_vcd",
                                    pv->name() + "_powerctrl_input_vcq",
                                    pv->name() + "_powerctrl_input_ircd",
                                    pv->name() + "_powerctrl_input_ircq"};
  logger->logAttribute(inputNames, pv->attribute("powerctrl_inputs"));
  std::vector<String> stateNames = {pv->name() + "_powerctrl_state_p",
                                    pv->name() + "_powerctrl_state_q",
                                    pv->name() + "_powerctrl_state_phid",
                                    pv->name() + "_powerctrl_state_phiq",
                                    pv->name() + "_powerctrl_state_gammad",
                                    pv->name() + "_powerctrl_state_gammaq"};
  logger->logAttribute(stateNames, pv->attribute("powerctrl_states"));
  std::vector<String> outputNames = {pv->name() + "_powerctrl_output_vsd",
                                     pv->name() + "_powerctrl_output_vsq"};
  logger->logAttribute(outputNames, pv->attribute("powerctrl_outputs"));

  // interface variables
  logger->logAttribute(pv->name() + "_v_intf", pv->attribute("v_intf"));
  logger->logAttribute(pv->name() + "_i_intf", pv->attribute("i_intf"));

  // additional variables
  logger->logAttribute(pv->name() + "_pll_output", pv->attribute("pll_output"));
  logger->logAttribute(pv->name() + "_vsref", pv->attribute("Vsref"));
  logger->logAttribute(pv->name() + "_vs", pv->attribute("Vs"));
}

} // namespace CIGREMV

namespace IEEE9 {
struct Gen {
  String Name;               // Name of Generator
  Real RatedPower;           // Rated Power of Generator
  Real RatedVoltage;         // Rated Voltage of Generator
  Real InitialPower;         // Initial Power of Generator
  Real InitialPowerReactive; // Initial Reactive Power of Generator
  Real InitialVoltage;       // Initial Voltage of Generator
  PowerflowBusType BusType;  // Bus Type of Generator

  // Generator Parameters
  Real Ra; // Armature Resistance (PU)
  Real Xa; // Armature Reactance (PU)

  Real Xd;             // Synchronous Reactance (PU)
  Real XdPrime;        // Transient Reactance (PU)
  Real XdDoublePrime;  // Sub-Transient Reactance (PU)
  Real Xq;             // Quadrature-Axis Reactance (PU)
  Real XqPrime;        // Quadrature-Axis Transient Reactance (PU)
  Real XqDoublePrime;  // Quadrature-Axis Sub-Transient Reactance (PU)
  Real Ld;             // Synchronous Inductance (PU)
  Real LdPrime;        // Transient Inductance (PU)
  Real LdDoublePrime;  // Sub-Transient Inductance (PU)
  Real Lq;             // Quadrature-Axis Inductance (PU)
  Real LqPrime;        // Quadrature-Axis Transient Inductance (PU)
  Real LqDoublePrime;  // Quadrature-Axis Sub-Transient Inductance (PU)
  Real H;              // Inertia Constant (s)
  Real D;              // Damping Coefficient (PU/PU)
  Real TdoPrime;       // Open-Circuit Field Time Constant (s)
  Real TdoDoublePrime; // Sub-Transient Time Constant (s)
  Real TqoPrime;
  Real TqoDoublePrime;

  Real Taa; // Accelerating Time Constant (s)
  Real L0;  // Leakage Reactance (PU)
};          // Generator Structure

struct Exciter {
  // Exciter (IEEE Type DC1A)
  Real KA;    // Voltage regulator gain (PU)
  Real TA;    // Voltage regulator time constant (s)
  Real VRmin; // Minimum exciter output limit (PU)
  Real VRmax; // Maximum exciter output limit (PU)
  Real KE;    // Exciter field proportional constant (PU)
  Real TE;    // Exciter field time constant (s)
  Real KF;    // Stabilizer feedback gain (PU)
  Real TF;    // Stabilizer feedback time constant (s)

  // Exciter (IEEE Type DC1A)
  Real EX1;   // Saturation point 1 for exciter (PU)
  Real S_EX1; // Saturation value at EX1 (PU)
  Real EX2;   // Saturation point 2 for exciter (PU)
  Real S_EX2; // Saturation value at EX2 (PU)
};            // Exciter Structure

struct Governor {
  // Governor Data (TGOV1)
  Real R;    // Speed droop (PU/PU)
  Real T1;   // Governor time constant (s)
  Real Vmax; // Maximum valve position (PU)
  Real Vmin; // Minimum valve position (PU)
  Real T2;   // Turbine time constant (s)
  Real T3;   // Servo time constant (s)
  Real Dt;   // Turbine damping coefficient (PU)
};           // Governor Structure

struct Load {
  String Name;
  Real RealPower;
  Real ReactivePower;
  Real BaseVoltage;
  Real Conductance;
  Real Susceptance;
}; // Load Structure

struct Line {
  String Name;
  Real ResistancePU;  // Resistance per unit
  Real ReactancePU;   // Reactance per unit
  Real SusceptancePU; // Susceptance per unit
  Real Resistance;    // Resistance
  Real Inductance;    // Inductance
  Real Capacitance;   // Capacitance
  Real Conductance;   // Conductance
  Real BaseVoltage;   // Base Voltage
};                    // Line Structure

struct Transformer {
  String Name;            // Name of the Transformer
  Real VoltageHVSide;     // High Voltage Side Voltage (in volts)
  Real VoltageLVSide;     // Low Voltage Side Voltage (in volts)
  Real RatedPower;        // Rated Power of the Transformer (in VA)
  Real Ratio;             // Turns Ratio (unitless)
  Real WindingConnection; // Winding Connection Type (0 for default)
  Real ResistancePU;      // Resistance per unit
  Real Resistance;        // Resistance (in ohms)
  Real ReactancePU;       // Reactance  per unit
  Real Inductance;        // Inductance (in henrys)
};                        // Transformer Structure

struct ScenarioConfig {
  // Network parameters
  Real Vnom = 230e3;
  Real nomFreq;
  Real nomOmega;
  Real baseMVA = 100e6;
  Real baseZ = Vnom * Vnom / baseMVA;

  // Generators
  Gen gen1;
  Gen gen2;
  Gen gen3;
  // Exciters
  Exciter exc1;
  Exciter exc2;
  Exciter exc3;
  // Governors
  Governor gov1;
  Governor gov2;
  Governor gov3;

  // Loads
  Load load5;
  Load load6;
  Load load8;

  // Lines
  Line line54;
  Line line64;
  Line line75;
  Line line96;
  Line line78;
  Line line89;

  // Transformers
  Transformer transf14;
  Transformer transf27;
  Transformer transf39;

  ScenarioConfig(Real nomFreq_ = 60)
      : nomFreq(nomFreq_), nomOmega(nomFreq_ * 2 * PI) {
    // Generator 1 (bus1)
    gen1.Name = "GEN1";
    gen1.RatedPower = 100e6;
    gen1.RatedVoltage = 16.5e3;
    gen1.InitialPower = 71.6e6;
    gen1.InitialPowerReactive = 27e6;
    gen1.InitialVoltage = 1.04 * gen1.RatedVoltage;
    gen1.BusType = PowerflowBusType::VD;

    gen1.Ra = 0.000125;
    gen1.Xa = 0.01460;
    gen1.L0 = gen1.Xa / (2 * PI * nomFreq);

    gen1.Xd = 0.146;
    gen1.XdPrime = 0.0608;
    gen1.XdDoublePrime = 0.06;

    gen1.Ld = gen1.Xd / (2 * PI * nomFreq);
    gen1.LdPrime = gen1.XdPrime / (2 * PI * nomFreq);
    gen1.LdDoublePrime = gen1.XdDoublePrime / (2 * PI * nomFreq);

    gen1.Xq = 0.1;
    gen1.XqPrime = 0.0969;
    gen1.XqDoublePrime = 0.06;

    gen1.Lq = gen1.Xq / (2 * PI * nomFreq);
    gen1.LqPrime = gen1.XqPrime / (2 * PI * nomFreq);
    gen1.LqDoublePrime = gen1.XqDoublePrime / (2 * PI * nomFreq);

    gen1.H = 23.64;
    gen1.D = 0.0;
    gen1.TdoPrime = 8.96;
    gen1.TdoDoublePrime = 0.01;
    gen1.TqoPrime = 0.310;
    gen1.TqoDoublePrime = 0.01;
    gen1.Taa = 0.0;

    // Exciter (IEEE Type DC1A)
    exc1.KA = 20.0;
    exc1.TA = 0.2;
    exc1.VRmin = -5.0;
    exc1.VRmax = 5.0;
    exc1.KE = 1.0;
    exc1.TE = 0.314;
    exc1.KF = 0.063;
    exc1.TF = 0.35;

    // Exciter (IEEE Type DC1A)
    exc1.EX1 = 3.3;
    exc1.S_EX1 = 0.6602;
    exc1.EX2 = 4.5;
    exc1.S_EX2 = 4.2662;

    // Governor Data (TGOV1)
    gov1.R = 0.05;
    gov1.T1 = 0.05;
    gov1.Vmax = 5.00;
    gov1.Vmin = -5.00;
    gov1.T2 = 2.1;
    gov1.T3 = 7.0;
    gov1.Dt = 0.0;

    // Generator 2 (bus2)
    gen2.Name = "GEN2";
    gen2.RatedPower = 100e6; //163e6;
    gen2.RatedVoltage = 18e3;
    gen2.InitialPower = 163e6;
    gen2.InitialPowerReactive = 6.7e6;
    gen2.InitialVoltage = 1.025 * gen2.RatedVoltage;
    gen2.BusType = PowerflowBusType::PV;

    gen2.Ra = 0.000125;
    gen2.Xa = 0.08958;
    gen2.L0 = gen2.Xa / (2 * PI * nomFreq);

    gen2.Xd = 0.8958;
    gen2.XdPrime = 0.1198;
    gen2.XdDoublePrime = 0.11;

    gen2.Ld = gen2.Xd / (2 * PI * nomFreq);
    gen2.LdPrime = gen2.XdPrime / (2 * PI * nomFreq);
    gen2.LdDoublePrime = gen2.XdDoublePrime / (2 * PI * nomFreq);

    gen2.Xq = 0.8645;
    gen2.XqPrime = 0.1969;
    gen2.XqDoublePrime = 0.11;

    gen2.Lq = gen2.Xq / (2 * PI * nomFreq);
    gen2.LqPrime = gen2.XqPrime / (2 * PI * nomFreq);
    gen2.LqDoublePrime = gen2.XqDoublePrime / (2 * PI * nomFreq);

    gen2.H = 6.40;
    gen2.D = 0.0;
    gen2.TdoPrime = 6.00;
    gen2.TdoDoublePrime = 0.01;
    gen2.TqoPrime = 0.535;
    gen2.TqoDoublePrime = 0.01;
    gen2.Taa = 0.0;

    // Exciter (IEEE Type DC1A)
    exc2.KA = 20.0;
    exc2.TA = 0.2;
    exc2.VRmin = -5.0;
    exc2.VRmax = 5.0;
    exc2.KE = 1.0;
    exc2.TE = 0.314;
    exc2.KF = 0.063;
    exc2.TF = 0.35;

    // Exciter (IEEE Type DC1A)
    exc2.EX1 = 3.3;
    exc2.S_EX1 = 0.6602;
    exc2.EX2 = 4.5;
    exc2.S_EX2 = 4.2662;

    // Governor Data (TGOV1)
    gov2.R = 0.05;
    gov2.T1 = 0.05;
    gov2.Vmax = 5.00;
    gov2.Vmin = -5.00;
    gov2.T2 = 2.1;
    gov2.T3 = 7.0;
    gov2.Dt = 0.0;

    // Generator 3 (bus3)
    gen3.Name = "GEN3";
    gen3.RatedPower = 100e6; //85e6;
    gen3.RatedVoltage = 13.8e3;
    gen3.InitialPower = 85e6;
    gen3.InitialPowerReactive = -10.9e6;
    gen3.InitialVoltage = 1.025 * gen3.RatedVoltage;
    gen3.BusType = PowerflowBusType::PV;
    gen3.Ra = 0.000125;
    gen3.Xa = 0.13125;
    gen3.L0 = gen3.Xa / (2 * PI * nomFreq);

    gen3.Xd = 1.3125;
    gen3.XdPrime = 0.1813;
    gen3.XdDoublePrime = 0.18;

    gen3.Ld = gen3.Xd / (2 * PI * nomFreq);
    gen3.LdPrime = gen3.XdPrime / (2 * PI * nomFreq);
    gen3.LdDoublePrime = gen3.XdDoublePrime / (2 * PI * nomFreq);

    gen3.Xq = 1.2578;
    gen3.XqPrime = 0.25;
    gen3.XqDoublePrime = 0.18;

    gen3.Lq = gen3.Xq / (2 * PI * nomFreq);
    gen3.LqPrime = gen3.XqPrime / (2 * PI * nomFreq);
    gen3.LqDoublePrime = gen3.XqDoublePrime / (2 * PI * nomFreq);

    gen3.H = 3.01;
    gen3.D = 0.0;
    gen3.TdoPrime = 5.89;
    gen3.TdoDoublePrime = 0.01;
    gen3.TqoPrime = 0.600;
    gen3.TqoDoublePrime = 0.01;
    gen3.Taa = 0.0;

    // Exciter (IEEE Type DC1A)
    exc3.KA = 20.0;
    exc3.TA = 0.2;
    exc3.VRmin = -5.0;
    exc3.VRmax = 5.0;
    exc3.KE = 1.0;
    exc3.TE = 0.314;
    exc3.KF = 0.063;
    exc3.TF = 0.35;

    // Exciter (IEEE Type DC1A)
    exc3.EX1 = 3.3;
    exc3.S_EX1 = 0.6602;
    exc3.EX2 = 4.5;
    exc3.S_EX2 = 4.2662;

    // Governor Data (TGOV1)
    gov3.R = 0.05;
    gov3.T1 = 0.05;
    gov3.Vmax = 5.00;
    gov3.Vmin = -5.00;
    gov3.T2 = 2.1;
    gov3.T3 = 7.0;
    gov3.Dt = 0.0;

    load5.Name = "LOAD5";
    load5.RealPower = 125e6;
    load5.ReactivePower = 50e6;
    load5.BaseVoltage = 230e3;
    load5.Conductance = load5.RealPower / std::pow(load5.BaseVoltage, 2);
    load5.Susceptance = -load5.ReactivePower / std::pow(load5.BaseVoltage, 2);

    load6.Name = "LOAD6";
    load6.RealPower = 90e6;
    load6.ReactivePower = 30e6;
    load6.BaseVoltage = 230e3;
    load6.Conductance = load6.RealPower / std::pow(load6.BaseVoltage, 2);
    load6.Susceptance = -load6.ReactivePower / std::pow(load6.BaseVoltage, 2);

    load8.Name = "LOAD8";
    load8.RealPower = 100e6;
    load8.ReactivePower = 35e6;
    load8.BaseVoltage = 230e3;
    load8.Conductance = load8.RealPower / std::pow(load8.BaseVoltage, 2);
    load8.Susceptance = -load8.ReactivePower / std::pow(load8.BaseVoltage, 2);

    line54.Name = "LINE54";
    line54.ResistancePU = 0.01005;
    line54.ReactancePU = 0.08521;
    line54.SusceptancePU = 1.0 / 5.688921;
    line54.Conductance = 0.0;
    line54.Resistance = line54.ResistancePU * baseZ;
    line54.Inductance = line54.ReactancePU * baseZ / nomOmega;
    line54.Capacitance = line54.SusceptancePU / baseZ / nomOmega;
    line54.BaseVoltage = 230e3;

    line64.Name = "LINE64";
    line64.ResistancePU = 0.017083;
    line64.ReactancePU = 0.092216;
    line64.SusceptancePU = 1.0 / 6.336801;
    line64.Conductance = 0.0;
    line64.Resistance = line64.ResistancePU * baseZ;
    line64.Inductance = line64.ReactancePU * baseZ / nomOmega;
    line64.Capacitance = line64.SusceptancePU / baseZ / nomOmega;
    line64.BaseVoltage = 230e3;

    line75.Name = "LINE75";
    line75.ResistancePU = 0.032533;
    line75.ReactancePU = 0.162281;
    line75.SusceptancePU = 1.0 / 3.28151;
    line75.Conductance = 0.0;
    line75.Resistance = line75.ResistancePU * baseZ;
    line75.Inductance = line75.ReactancePU * baseZ / nomOmega;
    line75.Capacitance = line75.SusceptancePU / baseZ / nomOmega;
    line75.BaseVoltage = 230e3;

    line96.Name = "LINE96";
    line96.ResistancePU = 0.039806;
    line96.ReactancePU = 0.171651;
    line96.SusceptancePU = 1.0 / 2.807618;
    line96.Conductance = 0.0;
    line96.Resistance = line96.ResistancePU * baseZ;
    line96.Inductance = line96.ReactancePU * baseZ / nomOmega;
    line96.Capacitance = line96.SusceptancePU / baseZ / nomOmega;
    line96.BaseVoltage = 230e3;

    line78.Name = "LINE78";
    line78.ResistancePU = 0.00853;
    line78.ReactancePU = 0.072127;
    line78.SusceptancePU = 1.0 / 6.717421;
    line78.Conductance = 0.0;
    line78.Resistance = line78.ResistancePU * baseZ;
    line78.Inductance = line78.ReactancePU * baseZ / nomOmega;
    line78.Capacitance = line78.SusceptancePU / baseZ / nomOmega;
    line78.BaseVoltage = 230e3;

    line89.Name = "LINE89";
    line89.ResistancePU = 0.011984;
    line89.ReactancePU = 0.10115;
    line89.SusceptancePU = 1.0 / 4.793121;
    line89.Conductance = 0.0;
    line89.Resistance = line89.ResistancePU * baseZ;
    line89.Inductance = line89.ReactancePU * baseZ / nomOmega;
    line89.Capacitance = line89.SusceptancePU / baseZ / nomOmega;
    line89.BaseVoltage = 230e3;

    transf14.Name = "TR14";
    transf14.VoltageLVSide = 16.5e3;
    transf14.VoltageHVSide = 230e3;
    transf14.RatedPower = 100e6;
    transf14.Ratio = transf14.VoltageLVSide / transf14.VoltageHVSide;
    transf14.WindingConnection = 0;
    transf14.ResistancePU = 0;
    transf14.Resistance = transf14.ResistancePU * baseZ;
    transf14.ReactancePU = 0.0576;
    transf14.Inductance = transf14.ReactancePU * baseZ / nomOmega;

    transf27.Name = "TR27";
    transf27.VoltageLVSide = 18e3;
    transf27.VoltageHVSide = 230e3;
    transf27.RatedPower = 100e6;
    transf27.Ratio = transf27.VoltageLVSide / transf27.VoltageHVSide;
    transf27.WindingConnection = 0;
    transf27.ResistancePU = 0;
    transf27.Resistance = transf27.ResistancePU * baseZ;
    transf27.ReactancePU = 0.0625;
    transf27.Inductance = transf27.ReactancePU * baseZ / nomOmega;
    ;

    transf39.Name = "TR39";
    transf39.VoltageLVSide = 13.8e3;
    transf39.VoltageHVSide = 230e3;
    transf39.RatedPower = 100e6;
    transf39.Ratio = transf39.VoltageLVSide / transf39.VoltageHVSide;
    transf39.WindingConnection = 0;
    transf39.ResistancePU = 0;
    transf39.Resistance = transf39.ResistancePU * baseZ;
    transf39.ReactancePU = 0.0586;
    transf39.Inductance = transf39.ReactancePU * baseZ / nomOmega;
    ;
  }
};
} // namespace IEEE9

} // namespace Grids

namespace Events {
std::shared_ptr<DPsim::SwitchEvent> createEventAddPowerConsumption(
    String nodeName, Real eventTime, Real additionalActivePower,
    SystemTopology &system, Domain domain, DPsim::DataLogger::Ptr logger) {

  // TODO: use base classes ph1
  if (domain == CPS::Domain::DP) {
    auto loadSwitch = DP::Ph1::Switch::make("Load_Add_Switch_" + nodeName,
                                            Logger::Level::debug);
    auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
    Real resistance = std::abs(connectionNode->initialSingleVoltage()) *
                      std::abs(connectionNode->initialSingleVoltage()) /
                      additionalActivePower;
    loadSwitch->setParameters(1e9, resistance);
    loadSwitch->open();
    system.addComponent(loadSwitch);
    system.connectComponentToNodes<Complex>(
        loadSwitch, {CPS::SimNode<Complex>::GND, connectionNode});
    logger->logAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
    return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
  } else if (domain == CPS::Domain::SP) {
    auto loadSwitch = SP::Ph1::Switch::make("Load_Add_Switch_" + nodeName,
                                            Logger::Level::debug);
    auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
    Real resistance = std::abs(connectionNode->initialSingleVoltage()) *
                      std::abs(connectionNode->initialSingleVoltage()) /
                      additionalActivePower;
    loadSwitch->setParameters(1e9, resistance);
    loadSwitch->open();
    system.addComponent(loadSwitch);
    system.connectComponentToNodes<Complex>(
        loadSwitch, {CPS::SimNode<Complex>::GND, connectionNode});
    logger->logAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
    return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
  } else {
    return nullptr;
  }
}

std::shared_ptr<DPsim::SwitchEvent3Ph> createEventAddPowerConsumption3Ph(
    String nodeName, Real eventTime, Real additionalActivePower,
    SystemTopology &system, Domain domain, DPsim::DataLogger::Ptr logger) {

  // TODO: use base classes ph3
  if (domain == CPS::Domain::EMT) {
    auto loadSwitch = EMT::Ph3::Switch::make("Load_Add_Switch_" + nodeName,
                                             Logger::Level::debug);
    auto connectionNode = system.node<CPS::SimNode<Real>>(nodeName);
    Real resistance = std::abs(connectionNode->initialSingleVoltage()) *
                      std::abs(connectionNode->initialSingleVoltage()) /
                      additionalActivePower;
    loadSwitch->setParameters(Matrix::Identity(3, 3) * 1e9,
                              Matrix::Identity(3, 3) * resistance);
    loadSwitch->openSwitch();
    system.addComponent(loadSwitch);
    system.connectComponentToNodes<Real>(
        loadSwitch,
        {CPS::SimNode<Real>::GND, system.node<CPS::SimNode<Real>>(nodeName)});
    logger->logAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
    return DPsim::SwitchEvent3Ph::make(eventTime, loadSwitch, true);
  } else {
    return nullptr;
  }
}
} // namespace Events
} // namespace Examples
} // namespace CIM
} // namespace CPS
