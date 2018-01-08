/** Ideal Transformer DP
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

#include "DP_Transformer.h"

using namespace DPsim;

Components::DP::Transformer::Transformer(String name, Int node1, Int node2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance)
	: Base(name, node1, node2)
{
	mRatioAbs = ratioAbs;
	mRatioPhase = ratioPhase;
	mRatio = std::polar(ratioAbs, ratioPhase);
	mResistance = resistance;
	mInductance = inductance;
	mNumVirtualNodes = 2;
	mVirtualNodes = { 0, 0 };

	if (resistance > 0.0001) {
		mNumVirtualNodes = 3;
		mVirtualNodes = { 0, 0, 0 };
	}
}

// TODO: implement RX losses
void Components::DP::Transformer::init(Real om, Real dt)
{
	mInductor = std::make_shared<Components::DP::Inductor>(mName + "_ind", mNode1, mVirtualNodes[0], mInductance);
	mInductor->init(om, dt);

}

void Components::DP::Transformer::applySystemMatrixStamp(SystemModel& system)
{
	if (mNode1 >= 0) {
		system.setCompSystemMatrixElement(mVirtualNodes[0], mVirtualNodes[1], -1.0, 0);
		system.setCompSystemMatrixElement(mVirtualNodes[1], mVirtualNodes[0], 1.0, 0);
	}
	if (mNode2 >= 0) {
		system.setCompSystemMatrixElement(mNode2, mVirtualNodes[1], mRatio.real(), mRatio.imag());
		system.setCompSystemMatrixElement(mVirtualNodes[1], mNode2, -mRatio.real(), -mRatio.imag());
	}

	if (mNumVirtualNodes == 2) {
		// Add inductive part to system matrix
		mInductor->applySystemMatrixStamp(system);
	}
}

void Components::DP::Transformer::step(SystemModel& system, Real time)
{
	mInductor->step(system, time);
}

void Components::DP::Transformer::postStep(SystemModel& system)
{
	mInductor->postStep(system);
}
