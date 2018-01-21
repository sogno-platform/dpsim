/** PQ Load
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

#include "DP_Load_PQ.h"

using namespace DPsim;

Components::DP::PQLoad::PQLoad(String name, Int node, Real activePower, Real reactivePower,
	Real volt, Real angle, Logger::Level loglevel, Bool decrementNodes)
	: Component(name, node, 0)
{
	// we need the system frequency to calculate the impedance, so we initialize
	// it with the dummy value of 1+j1 here for now
	mActivePower = activePower;
	mReactivePower = reactivePower;
	mSvVoltage = volt;
	attrMap["activePower"]   = { Attribute::Real, &mActivePower };
	attrMap["reactivePower"] = { Attribute::Real, &mReactivePower };
	attrMap["svVoltage"]     = { Attribute::Real, &mSvVoltage };
}

void Components::DP::PQLoad::initialize(SystemModel& system) {
	Real abs = mActivePower*mActivePower + mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	mReactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = mReactance / system.getOmega();

	inductor = std::make_shared<Components::DP::Inductor>(mName + "_ind", mNode1, mNode2, mInductance, mLogLevel);
	resistor = std::make_shared<Components::DP::Resistor>(mName + "_res", mNode1, mNode2, mResistance, mLogLevel);
	inductor->initialize(system);
	resistor->initialize(system);
}

void Components::DP::PQLoad::applySystemMatrixStamp(SystemModel& system)
{
	// Add resistive part to system matrix
	resistor->applySystemMatrixStamp(system);

	// Add inductive part to system matrix
	inductor->applySystemMatrixStamp(system);
}

void Components::DP::PQLoad::step(SystemModel& system, Real time)
{
	inductor->step(system, time);
}

void Components::DP::PQLoad::postStep(SystemModel& system)
{
	inductor->postStep(system);
}

Complex Components::DP::PQLoad::getCurrent(const SystemModel& system)
{
	return inductor->getCurrent(system);
}
