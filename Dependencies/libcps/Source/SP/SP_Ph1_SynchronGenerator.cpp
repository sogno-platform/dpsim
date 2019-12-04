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

SP::Ph1::SynchronGenerator::SynchronGenerator(String uid, String name,
	PowerflowBusType powerflowBusType,
	Logger::Level logLevel) : PowerComponent<Complex>(uid, name, logLevel) {

    setTerminalNumber(1);

	mPowerflowBusType = powerflowBusType;
    mSLog->info("Create {} of type {}", name, this->type());
};

SP::Ph1::SynchronGenerator::SynchronGenerator(String uid, String name, Real power, Real maxQ,
    Real vSetPoint, Real ratedU, Real ratedS, PowerflowBusType powerflowBusType, Logger::Level logLevel)
	: SynchronGenerator(uid, name, powerflowBusType, logLevel){
    mRatedS = ratedS;
    mRatedU = ratedU;

    switch (powerflowBusType)
    {
        case CPS::PowerflowBusType::PV:
            mSLog->info("Create PVNode as member of {}.", name);
            mPV = std::make_shared<PVNode>(uid, name, power, vSetPoint, maxQ, ratedU, ratedS, logLevel);
            mSLog->info("Power={} [W] Voltage={} [pu]", power, vSetPoint / ratedU);
            break;
        case CPS::PowerflowBusType::PQ:
            mSLog->info("Setting Synchronous Generator as PQNode is currently not supported.");
            break;
        case CPS::PowerflowBusType::VD:
            mSLog->info("Create VDNode as member of {}.", name);
            mVD = std::make_shared<VDNode>(uid, name, vSetPoint / ratedU, logLevel);
            mSLog->info("Power={} [W] Voltage={} [pu]", power, vSetPoint);
            break;
        default:
            throw std::invalid_argument(" Invalid power flow bus type ");
            break;
    }
};


void SP::Ph1::SynchronGenerator::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {

    mPowerflowBusType = powerflowBusType;
    switch (powerflowBusType)
    {
    case CPS::PowerflowBusType::PV:
        break;
    case CPS::PowerflowBusType::PQ:
        mSLog->info(" Setting Synchronous Generator as PQNode is currently not supported.");
        break;
    case CPS::PowerflowBusType::VD:
        mVD = std::make_shared<VDNode>(mUID, mName, mPV->attribute<Real>("V_set_pu")->get(), mLogLevel);
        break;
    case CPS::PowerflowBusType::None:
        break;
    default:
        throw std::invalid_argument(" Invalid power flow bus type ");
        break;
    }

}


