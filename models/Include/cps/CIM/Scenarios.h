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
        // network configuration
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
        Real KpPLL = 0.25;
        Real KiPLL = 2;
        Real KpPowerCtrl = 0.001;
        Real KiPowerCtrl = 0.08;
        Real KpCurrCtrl = 0.3;
        Real KiCurrCtrl = 10;
        Real OmegaCutoff = 2 * PI * systemFrequency;

        // PV filter parameters
        Real Lf = 0.002;
        Real Cf = 789.3e-6;
        Real Rf = 0.01;
        Real Rc = 0.05;

        // PV connection transformer parameters
        Real transformerNominalPower = 5e6;
        Real transformerInductance = 0.928e-3;

        // Further parameters
        Real systemOmega = 2 * PI * systemFrequency;
    };

    template <typename VarType>
    void addInvertersToCIGREMV(SystemTopology& system, CIGREMV::ScenarioConfig conf, Domain domain) {
        // add PVs to network topology
        for (Int n = 3; n <= 11; n++) {	
        // get connection node
        typename SimNode<VarType>::Ptr connectionNode;
        String connectionNodeName = "N" + std::to_string(n);
        for (auto sysNode : system.mNodes) {
            if (sysNode->name() == connectionNodeName) {
                connectionNode = std::dynamic_pointer_cast<SimNode<VarType>>(sysNode);				
                break;
            }
        }
        // add PV to connection node
        Real pvActivePower = conf.pvUnitNominalPower*conf.numberPVUnitsPerPlant;
        Real pvReactivePower = sqrt(std::pow(pvActivePower / conf.pvUnitPowerFactor, 2) - std::pow(pvActivePower, 2));

        // TODO: cast to BaseAverageVoltageSourceInverter and move set functions out of case distinction
        typename SimPowerComp<VarType>::Ptr pv;
        if (domain == Domain::SP) {
            pv = SP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
            std::dynamic_pointer_cast<SP::Ph1::AvVoltageSourceInverterDQ>(pv)->setParameters(conf.systemOmega, Complex(conf.pvUnitNominalVoltage, 0), pvActivePower, pvReactivePower);
            std::dynamic_pointer_cast<SP::Ph1::AvVoltageSourceInverterDQ>(pv)->setTransformerParameters(conf.systemNominalVoltage, conf.pvUnitNominalVoltage, conf.transformerNominalPower, conf.systemNominalVoltage/conf.pvUnitNominalVoltage, 0, 0, conf.transformerInductance, conf.systemOmega);
        } else if (domain == Domain::DP) {
            pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv_" + connectionNode->name(), "pv_" + connectionNode->name(), Logger::Level::debug, true);
            std::dynamic_pointer_cast<DP::Ph1::AvVoltageSourceInverterDQ>(pv)->setParameters(conf.systemOmega, Complex(conf.pvUnitNominalVoltage, 0), pvActivePower, pvReactivePower);
            std::dynamic_pointer_cast<DP::Ph1::AvVoltageSourceInverterDQ>(pv)->setControllerParameters(conf.KpPLL, conf.KiPLL, conf.KpPowerCtrl, conf.KiPowerCtrl, conf.KpCurrCtrl, conf.KiCurrCtrl, conf.OmegaCutoff);
            std::dynamic_pointer_cast<DP::Ph1::AvVoltageSourceInverterDQ>(pv)->setFilterParameters(conf.Lf, conf.Cf, conf.Rf, conf.Rc);
            std::dynamic_pointer_cast<DP::Ph1::AvVoltageSourceInverterDQ>(pv)->setTransformerParameters(conf.systemNominalVoltage, conf.pvUnitNominalVoltage, conf.transformerNominalPower, conf.systemNominalVoltage/conf.pvUnitNominalVoltage, 0, 0, conf.transformerInductance, conf.systemOmega);
        }
        
        system.addComponent(pv);
        system.connectComponentToNodes<VarType>(pv, { connectionNode });
        }
    }
}
}
}
}
