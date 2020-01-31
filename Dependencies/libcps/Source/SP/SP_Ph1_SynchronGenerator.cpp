/**
 * @file
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_SynchronGenerator.h>

using namespace CPS;

SP::Ph1::SynchronGenerator::SynchronGenerator(String uid, String name, Logger::Level logLevel)
 : PowerComponent<Complex>(uid, name, logLevel) {

    mSLog->info("Create {} of type {}", name, this->type());
    mSLog->flush();

    setTerminalNumber(1);
    
    addAttribute<Real>("P_set", &mSetPointActivePower, Flags::read | Flags::write);
    addAttribute<Real>("V_set", &mSetPointVoltage, Flags::read | Flags::write);
    addAttribute<Real>("P_set_pu", &mSetPointActivePowerPerUnit, Flags::read | Flags::write);
    addAttribute<Real>("V_set_pu", &mSetPointVoltagePerUnit, Flags::read | Flags::write);
};

void SP::Ph1::SynchronGenerator::setParameters(Real ratedApparentPower, Real ratedVoltage, Real setPointActivePower, Real setPointVoltage, Real maximumReactivePower, PowerflowBusType powerflowBusType) {
	mRatedApparentPower = ratedApparentPower;
    mRatedVoltage = ratedVoltage;
    mSetPointActivePower = setPointActivePower;
    mSetPointVoltage = setPointVoltage;
    mMaximumReactivePower = maximumReactivePower;
    mPowerflowBusType = powerflowBusType;

	mSLog->info("Rated Apparent Power={} [VA] Rated Voltage={} [V]", mRatedApparentPower, mRatedVoltage);
    mSLog->info("Active Power Set Point={} [W] Voltage Set Point={} [V]", mSetPointActivePower, mSetPointVoltage);
    mSLog->info("Maximum Reactive Power={} [VAr]", mMaximumReactivePower);
	mSLog->flush();
}

// #### Powerflow section ####
void SP::Ph1::SynchronGenerator::setBaseVoltage(Real baseVoltage) {
    mBaseVoltage = baseVoltage;
}

void SP::Ph1::SynchronGenerator::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	mSLog->info("#### Calculate Per Unit Parameters for {}", mName); 
	mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
    mSLog->info("Base Power={} [VA]  Base Omega={} [1/s]", mBaseApparentPower, mBaseOmega);

	mSetPointActivePowerPerUnit = mSetPointActivePower/mBaseApparentPower;
	mSetPointVoltagePerUnit = mSetPointVoltage/mBaseVoltage;
	mSLog->info("Active Power Set Point={} [pu] Voltage Set Point={} [pu]", mSetPointActivePowerPerUnit, mSetPointVoltagePerUnit);
	mSLog->flush();
}

void SP::Ph1::SynchronGenerator::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {
    switch (powerflowBusType)
    {
    case CPS::PowerflowBusType::PV:
        mPowerflowBusType = powerflowBusType;
        break;
    case CPS::PowerflowBusType::PQ:
        throw std::invalid_argument("Setting Synchronous Generator as PQNode is currently not supported.");
        break;
    case CPS::PowerflowBusType::VD:
        mPowerflowBusType = powerflowBusType;
        break;
    case CPS::PowerflowBusType::None:
		break;
    default:
        throw std::invalid_argument(" Invalid power flow bus type ");
        break;
    }
}


