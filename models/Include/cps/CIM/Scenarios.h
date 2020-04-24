/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Definitions.h>
#include <cps/SystemTopology.h>
#include <cps/Components.h>

#pragma once

namespace CPS {
namespace CIM {
namespace Scenarios {
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
        Real scaling_P = 10.0;
        Real scaling_I = 1000.0;

        Real KpPLL = 0.25/scaling_P;
        Real KiPLL = 2/scaling_I;
        Real KpPowerCtrl = 0.001/scaling_P;
        Real KiPowerCtrl = 0.08/scaling_I;
        Real KpCurrCtrl = 0.3/scaling_P;
        Real KiCurrCtrl = 10/scaling_I;
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
    };

    void addInvertersToCIGREMV(SystemTopology& system, CIGREMV::ScenarioConfig scenario, Domain domain) {
        Real pvActivePower = scenario.pvUnitNominalPower*scenario.numberPVUnitsPerPlant;
        Real pvReactivePower = sqrt(std::pow(pvActivePower / scenario.pvUnitPowerFactor, 2) - std::pow(pvActivePower, 2));

        // add PVs to network topology
        for (Int n = 3; n <= 11; n++) {	
            // TODO: cast to BaseAverageVoltageSourceInverter and move set functions out of case distinction
            if (domain == Domain::SP) {
                SimNode<Complex>::Ptr connectionNode = system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
                auto pv = SP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, Complex(scenario.pvUnitNominalVoltage, 0), pvActivePower, pvReactivePower);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance, scenario.systemOmega);
                system.addComponent(pv);
                system.connectComponentToNodes<Complex>(pv, { connectionNode });
            } else if (domain == Domain::DP) {
                SimNode<Complex>::Ptr connectionNode = system.node<CPS::SimNode<Complex>>("N" + std::to_string(n));
                auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, Complex(scenario.pvUnitNominalVoltage, 0), pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, 0, scenario.transformerInductance, scenario.systemOmega);
                system.addComponent(pv);
                system.connectComponentToNodes<Complex>(pv, { connectionNode });
            } else if (domain == Domain::EMT) {
                SimNode<Real>::Ptr connectionNode = system.node<CPS::SimNode<Real>>("N" + std::to_string(n));
                auto pv = EMT::Ph3::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
                pv->setParameters(scenario.systemOmega, Complex(scenario.pvUnitNominalVoltage, 0), pvActivePower, pvReactivePower);
                pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
                pv->setFilterParameters(Matrix::Identity(3,3)*scenario.Lf, Matrix::Identity(3,3)*scenario.Cf, Matrix::Identity(3,3)*scenario.Rf, Matrix::Identity(3,3)*scenario.Rc);
                pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvUnitNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvUnitNominalVoltage, 0, Matrix::Zero(3,3), Matrix::Identity(3,3)*scenario.transformerInductance, scenario.systemOmega);
                system.addComponent(pv);
                system.connectComponentToNodes<Real>(pv, { connectionNode });
            }
        }
    }

    std::shared_ptr<DPsim::SwitchEvent> createEventAddPowerConsumption(String nodeName, Real eventTime, Real additionalActivePower, SystemTopology& system, Domain domain) {
        
        // TODO: use base classes ph1
        if (domain == CPS::Domain::DP) {
            auto loadSwitch = DP::Ph1::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            auto connectionNode = system.node<CPS::SimNode<Complex>>(nodeName);
            Real resistance = std::abs(connectionNode->initialSingleVoltage())*std::abs(connectionNode->initialSingleVoltage())/additionalActivePower;
            loadSwitch->setParameters(1e9, resistance);
            loadSwitch->open();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Complex>(loadSwitch, { CPS::SimNode<Complex>::GND, connectionNode});
            return DPsim::SwitchEvent::make(eventTime, loadSwitch, true);
        } else {
            return nullptr;
        }
    }

    std::shared_ptr<DPsim::SwitchEvent3Ph> createEventAddPowerConsumption3Ph(String nodeName, Real eventTime, Real addedPowerAsResistance, SystemTopology& system, Domain domain) {
        
        // TODO: use base classes ph3
         if (domain == CPS::Domain::EMT) {
            auto loadSwitch = EMT::Ph3::Switch::make("Load_Add_Switch_" + nodeName, Logger::Level::debug);
            loadSwitch->setParameters(Matrix::Identity(3,3)*1e9, Matrix::Identity(3,3)*addedPowerAsResistance);
            loadSwitch->openSwitch();
            system.addComponent(loadSwitch);
            system.connectComponentToNodes<Real>(loadSwitch, { CPS::SimNode<Real>::GND, system.node<CPS::SimNode<Real>>(nodeName) });
            return DPsim::SwitchEvent3Ph::make(eventTime, loadSwitch, true);
        } else {
            return nullptr;
        }
    }

    void logPVAttributes(DPsim::DataLogger::Ptr logger, CPS::SimPowerComp<Complex>::Ptr pv) {
        // state variables
        logger->addAttribute(pv->name() + "_state_" + "theta", pv->attribute("theta"));
        logger->addAttribute(pv->name() + "_state_" + "phipll", pv->attribute("phipll"));
        logger->addAttribute(pv->name() + "_state_" + "p", pv->attribute("p"));
        logger->addAttribute(pv->name() + "_state_" + "q", pv->attribute("q"));
        logger->addAttribute(pv->name() + "_state_" + "phid", pv->attribute("phid"));
        logger->addAttribute(pv->name() + "_state_" + "phiq", pv->attribute("phiq"));
        logger->addAttribute(pv->name() + "_state_" + "gammad", pv->attribute("gammad"));
        logger->addAttribute(pv->name() + "_state_" + "gammaq", pv->attribute("gammaq"));

        // input variables
        logger->addAttribute(pv->name() + "_input_" + "Vcdq", pv->attribute("Vcdq"));
        logger->addAttribute(pv->name() + "_input_" + "Ircdq", pv->attribute("Ircdq"));

        // output variables
        logger->addAttribute(pv->name() + "_output_" + "Vsdq", pv->attribute("Vsdq"));

        // interface variables
        logger->addAttribute(pv->name() + "_v_intf", pv->attribute("v_intf"));
        logger->addAttribute(pv->name() + "_i_intf", pv->attribute("i_intf"));
    }
	

}
}
}
}
