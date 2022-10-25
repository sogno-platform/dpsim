/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim-models/Components.h>

#pragma once

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
}

namespace Exciter{
    // Excitation system parameters (IEEE Type DC1 - simplified)
    // from M. Eremia, "Handbook of Electrical Power System Dynamics", 2013, p.96 and 106
    // voltage-regulator

    CPS::Base::ExciterParameters getExciterEremia() {
        CPS::Base::ExciterParameters ExcitationSystemEremia;    
        ExcitationSystemEremia.Ka = 46;
        ExcitationSystemEremia.Ta = 0.06;
        // exciter
        ExcitationSystemEremia.Kef = -0.0435;
        ExcitationSystemEremia.Tef = 0.46;
        // stabilizing feedback
        ExcitationSystemEremia.Kf = 0.1;
        ExcitationSystemEremia.Tf = 1;
        // voltage transducer
        ExcitationSystemEremia.Tr = 0.02;
        // saturation function coefficients
        ExcitationSystemEremia.Aef = 0.33;
        ExcitationSystemEremia.Bef = 0.1;
        
        ExcitationSystemEremia.MaxVr = 1.0;
        ExcitationSystemEremia.MinVr = -0.9;

        return ExcitationSystemEremia;
    }

    struct ExcitationKundur {
        /// Exciter model used in Kundurs. It is a very simplified version of a thyristor 
	    /// exciter (ST1 type) without transient gain reduction or derivative feedback 
	    /// (only proportional block + terminal voltage transducer) 
	    /// Ref.: Kundur,  Power System Stability and Control, p. 865
        Real Ka = 200;
        // voltage transducer
        Real Tr = 0.02;
    };
}

namespace PowerSystemStabilizer {
    struct PSSType2PSAT {
        // Power system stabilizer type 2 
        // Taken from from PSAT - example d_anderson_farmer Gen2

        /// Stabilizer gain for active power (pu/pu)
		Real Kp = 0;
		/// Stabilizer gain for bus voltage magnitude (pu/pu)
		Real Kv = 0;
		/// Stabilizer gain for omega gain (pu/pu)
		Real Kw = 15;
		/// First stabilizer time constant (s)
		Real T1 = 0.1;
		/// Second stabilizer time constant (s)
		Real T2 = 0.01;
		/// Thrid stabilizer time constant (s)
		Real T3 = 0.12;
		/// Fourth stabilizer time constant (s)
		Real T4 = 0.01;
		/// Max stabilizer output signal (pu)
		Real Vs_max = 0.1;
		/// Min stabilizer output signal (pu)
		Real Vs_min = -0.1;
		/// Wash-out time constant (s)
		Real Tw = 10;
    };

    struct PSSKundur {
        /// Power system stabilizer consisting of three blocks: a phase compensation
        /// block, a signal washout block, and a gain block
        /// Ref: Kundur, Power System Stability and Control, p. 865

        /// Stabilizer gain for active power (pu/pu)
		Real Kp = 0;
		/// Stabilizer gain for bus voltage magnitude (pu/pu)
		Real Kv = 0;
		/// Stabilizer gain for omega gain (pu/pu)
		Real Kw = 9.5;
		/// First stabilizer time constant (s)
		Real T1 = 0.154;
		/// Second stabilizer time constant (s)
		Real T2 = 0.033;
		/// Max stabilizer output signal (pu)
		Real Vs_max = 0.2;
		/// Min stabilizer output signal (pu)
		Real Vs_min = -0.2;
		/// Wash-out time constant (s)
		Real Tw = 1.41;
    };
}

namespace TurbineGovernor {
    struct GovernorKundur {
        // Turbine model parameters (tandem compound single reheat steam turbine, fossil-fuelled)
        // from P. Kundur, "Power System Stability and Control", 1994, p. 427
        Real Ta_t = 0.3;    // T_CH
        Real Fa = 0.3;      // F_HP
        Real Tb = 7.0;      // T_RH
        Real Fb = 0.3;      // F_IP
        Real Tc = 0.5;      // T_CO
        Real Fc = 0.4;      // F_LP

        // Governor parameters (mechanical-hydraulic control)
        // from P. Kundur, "Power System Stability and Control", 1994, p. 437
        Real Kg = 20;       // 5% droop
        Real Tsr = 0.1;
        Real Tsm = 0.3;
    };

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
}
}

namespace Grids {
namespace CIGREHVEuropean {
    struct LineParameters {
        // 220 kV
        Real Vnom = 220e3;
        Real nomFreq = 50;
        Real nomOmega= nomFreq* 2*PI;
        //PiLine parameters (given in [p.u/km] with Z_base= 529ohm)
        Real lineResistancePerKm = 1.35e-4  * 529;
        Real lineReactancePerKm = 8.22e-4  * 529;
        Real lineSusceptancePerKm = 1.38e-3  / 529;
        Real lineConductancePerKm = 0;
    };
}
namespace CIGREHVAmerican {
    struct LineParameters {
        // 230 kV
        Real Vnom = 230e3;
        Real nomFreq = 60;
        Real nomOmega= nomFreq* 2*PI;
        //PiLine parameters (given in [p.u/km] with Z_base= 529ohm)
        Real lineResistancePerKm = 1.27e-4 * 529;
        Real lineReactancePerKm = 9.05e-4 * 529;
        Real lineSusceptancePerKm = 1.81e-3 / 529;
        Real lineConductancePerKm = 0;
    };
}

namespace KundurExample1 {
    // P. Kundur, "Power System Stability and Control", Example 13.2, pp. 864-869.
    struct Network {
        Real nomVoltage = 400e3;
        Real nomFreq = 60;
        Real nomOmega = nomFreq* 2*PI;
    };

    struct Gen {
        Real nomPower = 2220e6;
        Real nomVoltage = 400e3;
        Real H = 3.5;
        Real XpdPU = 0.3;
        Real RsPU = 0;
        Real D = 1.0;
        Real initActivePower = 0.9 * nomPower;
        Real initMechPower = 0.9 * nomPower;
        Real setPointVoltage = nomVoltage;
    };

    struct Line1 {
        // Vnom = 400kV
        Real lineResistance = 0.0721;
        Real lineReactance = 36.0360;
        Real lineSusceptance = 0;
        Real lineConductance =0;
    };

    struct Line2 {
        // Vnom = 400kV
        Real lineResistance = 0.0721;
        Real lineReactance = 36.0360;
        Real lineSusceptance = 0;
        Real lineConductance =0;
    };

    struct Transf1 {
	    Real nomVoltageHV = 400e3;
        Real nomVoltageMV = 400e3;
        Real transformerResistance = 0; // referred to HV side
        Real transformerReactance = 10.8108; // referred to HV side
    };
}

namespace SMIB {
    struct ScenarioConfig {
        //-----------Network-----------//
        Real Vnom = 230e3;
        Real nomFreq = 60;
        Real nomOmega= nomFreq* 2*PI;
        //-----------Generator-----------//
        Real nomPower = 500e6;
        Real nomPhPhVoltRMS = 22e3;
        Real H = 5;
        Real Xpd=0.31;
        Real Rs = 0.003*0;
        Real D = 1.5;
        // Initialization parameters
        Real initMechPower= 300e6;
        Real initActivePower = 300e6;
        Real setPointVoltage=nomPhPhVoltRMS + 0.05*nomPhPhVoltRMS;
        //-----------Transformer-----------//
        Real t_ratio=Vnom/nomPhPhVoltRMS;
        //-----------Transmission Line-----------//
        // CIGREHVAmerican (230 kV)
        Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
        Real lineLeng=100;
        Real lineResistance = lineCIGREHV.lineResistancePerKm*lineLeng;
        Real lineInductance = lineCIGREHV.lineReactancePerKm/nomOmega*lineLeng;
        Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm/nomOmega*lineLeng;
        Real lineConductance =lineCIGREHV.lineConductancePerKm*lineLeng;
    };

    struct ScenarioConfig2 {
        //Scenario used to validate reduced order SG VBR models against PSAT (in SP domain)

        // General grid parameters
        Real VnomMV = 24e3;
        Real VnomHV = 230e3;
        Real nomFreq = 60;
        Real ratio = VnomMV/VnomHV;
        Real nomOmega= nomFreq * 2 * PI;

        // Generator parameters
        Real setPointActivePower = 300e6;
        Real setPointVoltage = 1.05*VnomMV;

        // CIGREHVAmerican (230 kV)
        Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
        Real lineLength = 100;
        Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio,2);
        Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength * std::pow(ratio,2) / nomOmega;
        Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength / std::pow(ratio,2) / nomOmega;
        Real lineConductance = 1e-15;

        // In PSAT SwitchClosed is equal to 1e-3 p.u.
        Real SwitchClosed = 0.001 * (24*24/555);
	    Real SwitchOpen = 1e6;
    };

    struct ScenarioConfig3 {
        //Scenario used to validate reduced order SG VBR models in the DP and EMT domain against the SP domain

        // General grid parameters
        Real VnomMV = 24e3;
        Real nomFreq = 60;
        Real nomOmega= nomFreq * 2 * PI;

        //-----------Generator-----------//
        Real setPointActivePower=300e6;
        Real mechPower = 300e6;
        Real initActivePower = 300e6;
        Real initReactivePower = 0;
        Real initVoltAngle = -PI / 2;
        Complex initComplexElectricalPower = Complex(initActivePower, initReactivePower);
        Complex initTerminalVolt = VnomMV * Complex(cos(initVoltAngle), sin(initVoltAngle));

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
        Real ratio = VnomMV/VnomHV;
        Real nomOmega= nomFreq * 2 * PI;

        // Generator parameters
        Real setPointActivePower = 300e6;
        Real setPointVoltage = 1.05*VnomMV;

        // CIGREHVAmerican (230 kV)
        Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
        Real lineLength = 100;
        Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio,2);
        Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength * std::pow(ratio,2) / nomOmega;
        Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength / std::pow(ratio,2) / nomOmega;
        Real lineConductance = 8e-2;

        // In PSAT SwitchClosed is equal to 1e-3 p.u.
        Real SwitchClosed = 0.1;
	    Real SwitchOpen = 1e6;
    };
}

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
        Real ratio = VnomMV/VnomHV;
        Real nomOmega= nomFreq * 2 * PI;

        // Generator parameters
        Real setPointActivePower = 300e6;
        Real setPointVoltage = 1.05*VnomMV;

        // CIGREHVAmerican (230 kV)
        Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
        Real lineLength = 100;
        Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength;
        Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength / nomOmega;
        Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength / nomOmega;
        Real lineConductance = 1.0491e-05; // Psnub 0.1% of 555MW

        // Switch for load step
        Real SwitchClosed = 529; // 100 MW load step
	    Real SwitchOpen = 9.1840e+07; // corresponds to 1e6 Ohms at MV level of 24kV
    };

    struct Transf1 {
	    Real nomVoltageHV = 230e3;
        Real nomVoltageMV = 24e3;
        Real transformerResistance = 0; // referred to HV side
        Real transformerReactance = 5.2900; // referred to HV side
        Real transformerNominalPower = 555e6;
    };
}

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
        Real ratio = VnomMV/VnomHV;
        Real nomOmega= nomFreq * 2 * PI;

        // Generator parameters
        Real setPointActivePower = 300e6;
        Real setPointVoltage = 1.05*VnomMV;

        // CIGREHVAmerican (230 kV)
        Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
        Real lineLength = 100;
        Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio,2);
        Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength * std::pow(ratio,2) / nomOmega;
        Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength / std::pow(ratio,2) / nomOmega;
        Real lineConductance = 0.0048; // Psnub 0.5% of 555MW

        // Load step
        Real loadStepActivePower = 100e6; 
    };
}

}
}

namespace ThreeBus {
    struct ScenarioConfig {

        //-----------Network-----------//
        Real Vnom = 230e3;
        Real nomFreq = 60;
        Real nomOmega= nomFreq* 2*PI;

        //-----------Generator 1 (bus1)-----------//
        Real nomPower_G1 = 300e6;
        Real nomPhPhVoltRMS_G1 = 25e3;
        Real nomFreq_G1 = 60;
        Real H_G1 = 6;
        Real Xpd_G1=0.3; //in p.u
        Real Rs_G1 = 0.003*0; //in p.u
        Real D_G1 = 1.5; //in p.u
        // Initialization parameters
        Real initActivePower_G1 = 270e6;
        Real initMechPower_G1 = 270e6;
        Real setPointVoltage_G1=nomPhPhVoltRMS_G1+0.05*nomPhPhVoltRMS_G1;

        //-----------Generator 2 (bus2)-----------//
        Real nomPower_G2 = 50e6;
        Real nomPhPhVoltRMS_G2 = 13.8e3;
        Real nomFreq_G2 = 60;
        Real H_G2 = 2;
        Real Xpd_G2=0.1; //in p.u
        Real Rs_G2 = 0.003*0; //in p.u
        Real D_G2 =1.5; //in p.u
        // Initialization parameters
        Real initActivePower_G2 = 45e6;
        Real initMechPower_G2 = 45e6;
        Real setPointVoltage_G2=nomPhPhVoltRMS_G2-0.05*nomPhPhVoltRMS_G2;

        //-----------Transformers-----------//
        Real t1_ratio=Vnom/nomPhPhVoltRMS_G1;
        Real t2_ratio=Vnom/nomPhPhVoltRMS_G2;

        //-----------Load (bus3)-----------
        Real activePower_L= 310e6;
        Real reactivePower_L= 150e6;

        // -----------Transmission Lines-----------//
        // CIGREHVAmerican (230 kV)
        Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
        //line 1-2 (180km)
        Real lineResistance12 = lineCIGREHV.lineResistancePerKm*180;
        Real lineInductance12 = lineCIGREHV.lineReactancePerKm/nomOmega*180;
        Real lineCapacitance12 = lineCIGREHV.lineSusceptancePerKm/nomOmega*180;
        Real lineConductance12 = lineCIGREHV.lineConductancePerKm*180;
        //line 1-3 (150km)
        Real lineResistance13 = lineCIGREHV.lineResistancePerKm*150;
        Real lineInductance13 = lineCIGREHV.lineReactancePerKm/nomOmega*150;
        Real lineCapacitance13 = lineCIGREHV.lineSusceptancePerKm/nomOmega*150;
        Real lineConductance13 = lineCIGREHV.lineConductancePerKm*150;
        //line 2-3 (80km)
        Real lineResistance23 = lineCIGREHV.lineResistancePerKm*80;
        Real lineInductance23 = lineCIGREHV.lineReactancePerKm/nomOmega*80;
        Real lineCapacitance23 = lineCIGREHV.lineSusceptancePerKm/nomOmega*80;
        Real lineConductance23 = lineCIGREHV.lineConductancePerKm*80;
    };
}

namespace SGIB {

    struct ScenarioConfig {
        Real systemFrequency = 50;
        Real systemNominalVoltage = 20e3;

        // Line parameters (R/X = 1)
        Real length = 5;
        Real lineResistance = 0.5 * length;
	    Real lineInductance = 0.5/314 * length;
        Real lineCapacitance = 50e-6/314 * length;

        // PV controller parameters
        Real scalingKp = 1;
        Real scalingKi = 0.1;

        Real KpPLL = 0.25*scalingKp;
        Real KiPLL = 2*scalingKi;
        Real KpPowerCtrl = 0.001*scalingKp;
        Real KiPowerCtrl = 0.08*scalingKi;
        Real KpCurrCtrl = 0.3*scalingKp;
        Real KiCurrCtrl = 10*scalingKi;
        Real OmegaCutoff = 2 * PI * systemFrequency;

        // Initial state values
        Real thetaPLLInit = 0; // only for debug
        Real phiPLLInit = 0; // only for debug
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
}

namespace CIGREMV {

    struct ScenarioConfig {
        Real systemFrequency = 50;
        Real systemNominalVoltage = 20e3;
        Real penetrationLevel = 1;
        Real totalLoad = 4319.1e3; // calculated total load in CIGRE MV left feeder (see CIM data)

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

		Real KpPLL = 0.25*scalingKp;
        Real KiPLL = 2*scalingKi;
        Real KpPowerCtrl = 0.001*scalingKp;
        Real KiPowerCtrl = 0.08*scalingKi;
        Real KpCurrCtrl = 0.3*scalingKp;
        Real KiCurrCtrl = 10*scalingKi;
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
        Real thetaPLLInit = 314.168313-systemOmega;
        Real phiPLLInit = 8e-06;
        Real pInit = 450000.716605;
        Real qInit = -0.577218;
        Real phi_dInit = 3854197405*scalingKi;
        Real phi_qInit = -3737*scalingKi;
        Real gamma_dInit = 128892668*scalingKi;
        Real gamma_qInit = 23068682*scalingKi;
    };

    void addInvertersToCIGREMV(SystemTopology& system, CIGREMV::ScenarioConfig scenario, Domain domain) {
        Real pvActivePower = scenario.pvUnitNominalPower*scenario.numberPVUnitsPerPlant;
        Real pvReactivePower = sqrt(std::pow(pvActivePower / scenario.pvUnitPowerFactor, 2) - std::pow(pvActivePower, 2));

        // add PVs to network topology
        for (Int n = 3; n <= 11; ++n) {
            // TODO: cast to BaseAverageVoltageSourceInverter and move set functions out of case distinction
            if (domain == Domain::SP) {
                SimNode<Complex>::Ptr connectionNode = system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
                auto pv = SP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage, pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance);
                pv->setInitialStateValues(scenario.pInit, scenario.qInit, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
                system.addComponent(pv);
                system.connectComponentToNodes<Complex>(pv, { connectionNode });
            } else if (domain == Domain::DP) {
                SimNode<Complex>::Ptr connectionNode = system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
                auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage, pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance);
                pv->setInitialStateValues(scenario.pInit, scenario.qInit, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
                system.addComponent(pv);
                system.connectComponentToNodes<Complex>(pv, { connectionNode });
            } else if (domain == Domain::EMT) {
                SimNode<Real>::Ptr connectionNode = system.node<CPS::SimNode<Real>>("N" + std::to_string(n));
                auto pv = EMT::Ph3::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, scenario.pvUnitNominalVoltage, pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance, scenario.systemOmega);
                pv->setInitialStateValues(scenario.pInit, scenario.qInit, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
                system.addComponent(pv);
                system.connectComponentToNodes<Real>(pv, { connectionNode });
            }
        }
    }

    void logPVAttributes(DPsim::DataLogger::Ptr logger, CPS::TopologicalPowerComp::Ptr pv) {

        // power controller
        std::vector<String> inputNames = {  pv->name() + "_powerctrl_input_pref", pv->name() + "_powerctrl_input_qref",
                                            pv->name() + "_powerctrl_input_vcd", pv->name() + "_powerctrl_input_vcq",
                                            pv->name() + "_powerctrl_input_ircd", pv->name() + "_powerctrl_input_ircq"};
        logger->logAttribute(inputNames, pv->attribute("powerctrl_inputs"));
        std::vector<String> stateNames = {  pv->name() + "_powerctrl_state_p", pv->name() + "_powerctrl_state_q",
                                            pv->name() + "_powerctrl_state_phid", pv->name() + "_powerctrl_state_phiq",
                                            pv->name() + "_powerctrl_state_gammad", pv->name() + "_powerctrl_state_gammaq"};
        logger->logAttribute(stateNames, pv->attribute("powerctrl_states"));
        std::vector<String> outputNames = {  pv->name() + "_powerctrl_output_vsd", pv->name() + "_powerctrl_output_vsq"};
        logger->logAttribute(outputNames, pv->attribute("powerctrl_outputs"));

        // interface variables
        logger->logAttribute(pv->name() + "_v_intf", pv->attribute("v_intf"));
        logger->logAttribute(pv->name() + "_i_intf", pv->attribute("i_intf"));

        // additional variables
        logger->logAttribute(pv->name() + "_pll_output", pv->attribute("pll_output"));
        logger->logAttribute(pv->name() + "_vsref", pv->attribute("Vsref"));
        logger->logAttribute(pv->name() + "_vs", pv->attribute("Vs"));
    }

}
}

namespace Events {
        std::shared_ptr<DPsim::SwitchEvent> createEventAddPowerConsumption(String nodeName, Real eventTime, Real additionalActivePower, SystemTopology& system, Domain domain, DPsim::DataLogger::Ptr logger) {

        // TODO: use base classes ph1
        if (domain == CPS::Domain::DP) {
            auto loadSwitch = DP::Ph1::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(1e9, resistance);
            loadSwitch->open();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Complex>(loadSwitch, { CPS::SimNode<Complex>::GND, connectionNode});
            logger->logAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
            return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
        } else if (domain == CPS::Domain::SP) {
            auto loadSwitch = SP::Ph1::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(1e9, resistance);
            loadSwitch->open();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Complex>(loadSwitch, { CPS::SimNode<Complex>::GND, connectionNode});
            logger->logAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
            return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
        } else {
            return nullptr;
        }
    }

    std::shared_ptr<DPsim::SwitchEvent3Ph> createEventAddPowerConsumption3Ph(String nodeName, Real eventTime, Real additionalActivePower, SystemTopology& system, Domain domain, DPsim::DataLogger::Ptr logger) {

        // TODO: use base classes ph3
         if (domain == CPS::Domain::EMT) {
            auto loadSwitch = EMT::Ph3::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Real>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(Matrix::Identity(3,3)*1e9, Matrix::Identity(3,3)*resistance);
            loadSwitch->openSwitch();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Real>(loadSwitch, { CPS::SimNode<Real>::GND, system.node<CPS::SimNode<Real>>(nodeName) });
            logger->logAttribute("switchedload_i", loadSwitch->attribute("i_intf"));
            return DPsim::SwitchEvent3Ph::make(eventTime, loadSwitch, true);
        } else {
            return nullptr;
        }
    }
}
}
}
}
