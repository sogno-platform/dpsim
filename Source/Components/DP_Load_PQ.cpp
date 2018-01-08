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

Components::DP::PQLoad::PQLoad(String name, Int node, Real activePower, Real reactivePower, Real volt, Real angle)
	: Base(name, node, 0)
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

void Components::DP::PQLoad::init(Real om, Real dt)
{
	Real abs = mActivePower*mActivePower + mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower/abs;
	mConductance = 1.0 / mResistance;
	mReactance = mSvVoltage*mSvVoltage*mReactivePower/abs;
	mInductance = mReactance / om;

	inductor = std::make_shared<Components::DP::Inductor>(mName + "_ind", mNode1, mNode2, mInductance);
	resistor = std::make_shared<Components::DP::Resistor>(mName + "_res", mNode1, mNode2, mResistance);
	inductor->init(om, dt);
	resistor->init(om, dt);
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

Complex Components::DP::PQLoad::getCurrent(SystemModel& system)
{
	return inductor->getCurrent(system);
}
