/** Ideal Transformer DP
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
* @license GNU General Public License (version 3)
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

#include "TransformerDP.h"

using namespace DPsim;

TransformerDP::TransformerDP(String name, Int node1, Int node2, Real ratioAbs, Real ratioPhase, Real resistance, Real reactance) : BaseComponent(name, node1, node2) {
	mRatioAbs = ratioAbs;
	mRatioPhase = ratioPhase;
	mRatio = std::polar(ratioAbs, ratioPhase);
	mResistance = resistance;
	mReactance = reactance;
	mNumVirtualNodes = 2;
	
	if (resistance > 0.000001) {
		mNumVirtualNodes = 3;
		mVirtualNodes = { 0, 0, 0 };
	}	
}

void TransformerDP::init(Real om, Real dt) {
	Real abs = mActivePower*mActivePower + mReactivePower*mReactivePower;
	mResistance = mSvVoltage*mSvVoltage*mActivePower / abs;
	mConductance = 1.0 / mResistance;
	mReactance = mSvVoltage*mSvVoltage*mReactivePower / abs;
	mInductance = mReactance / om;

	inductor = std::make_shared<InductorDP>(mName + "_ind", mNode1, mNode2, mInductance);
	resistor = std::make_shared<ResistorDP>(mName + "_res", mNode1, mNode2, mResistance);
	inductor->init(om, dt);
	resistor->init(om, dt);
}

void TransformerDP::applySystemMatrixStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.setCompSystemMatrixElement(mNode1, mVirtualNodes[0], -1.0, 0);
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode1, 1.0, 0);
	}
	if (mNode2 >= 0) {
		system.setCompSystemMatrixElement(mNode2, mVirtualNodes[0], mRatioRe, mRatioIm);
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode2, -mRatioRe, -mRatioIm);
	}

	if (mNumVirtualNodes == 2) {
		// Add inductive part to system matrix
		inductor->applySystemMatrixStamp(system);
	}
}
