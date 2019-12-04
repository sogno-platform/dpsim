/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/SP/SP_Ph1_VoltageSourceInverter.h>

using namespace CPS;

SP::Ph1::VoltageSourceInverter::VoltageSourceInverter(String uid, String name,
	PowerflowBusType powerflowBusType,
	Logger::Level logLevel) : PowerComponent<Complex>(uid, name, logLevel) {
	mNumTerminals = 1;
	mTerminals.resize(mNumTerminals, nullptr);

	mPowerflowBusType = powerflowBusType;

	// mLog.debug() << "Create " << name << " of type " << this->type() << std::endl;
}

SP::Ph1::VoltageSourceInverter::VoltageSourceInverter(String uid, String name, Real power, Real reactivePower,
	PowerflowBusType powerflowBusType,
	Logger::Level logLevel) : VoltageSourceInverter(uid, name, powerflowBusType, logLevel) {
	mPowerflowBusType = powerflowBusType;

	switch (powerflowBusType)
	{
	case CPS::PowerflowBusType::PQ:
		mPQ = std::make_shared<PQNode>(mUID, mName, power,
			reactivePower, mLogLevel);
		break;
	default:
		// mLog.debug() << " Power flow bus type other than PQ for inverter were not implemented. " << std::endl;
		break;
	}
};


void SP::Ph1::VoltageSourceInverter::modifyPowerFlowBusType(PowerflowBusType powerflowBusType) {

	mPowerflowBusType = powerflowBusType;
	switch (powerflowBusType)
	{
	case CPS::PowerflowBusType::PV:
		throw std::invalid_argument(" inverters currently cannot be set as PV bus.");
		break;
	case CPS::PowerflowBusType::PQ:
		mPQ = std::make_shared<CPS::SP::Ph1::PQNode>(mUID, mName, mLogLevel);
		break;
	case CPS::PowerflowBusType::VD:
		throw std::invalid_argument(" inverters currently cannot be set as VD bus. ");
		break;
	case CPS::PowerflowBusType::None:
		break;
	default:
		throw std::invalid_argument(" Invalid power flow bus type ");
		break;
	}
}
