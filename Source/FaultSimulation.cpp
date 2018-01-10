/** Simulation with a configurable fault
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

#include "FaultSimulation.h"

using namespace DPsim;

void FaultSimulation::clearFault(Int Node1, Int Node2, Int Node3)
{
	if (mSystemModel.getSimType() != SimulationType::EMT)
		return;

	ClearingFault = true;

	mIfa = getRightSideVector()(Node1 - 1);
	mIfb = getRightSideVector()(Node2 - 1);
	mIfc = getRightSideVector()(Node3 - 1);

	if (FirstTime == true) {
		mIfa_hist = mIfa;
		mIfb_hist = mIfb;
		mIfc_hist = mIfc;

		FirstTime = false;
	}

	if (std::signbit(mIfa) != std::signbit(mIfa_hist) && !aCleared) {
		mComponents.erase(mComponents.begin() + 1);
		addSystemTopology(mComponents);
		switchSystemMatrix(mSwitchEventVector.size() + NumClearedPhases);
		NumClearedPhases++;
		aCleared = true;
	}

	if (std::signbit(mIfb) != std::signbit(mIfb_hist) && !bCleared) {
		mComponents.erase(mComponents.begin() + 2);
		addSystemTopology(mComponents);
		switchSystemMatrix(mSwitchEventVector.size() + NumClearedPhases);
		NumClearedPhases++;
		bCleared = true;
	}

	if (std::signbit(mIfc) != std::signbit(mIfc_hist) && !cCleared) {
		mComponents.erase(mComponents.begin() + 1);
		addSystemTopology(mComponents);
		switchSystemMatrix(mSwitchEventVector.size() + NumClearedPhases);
		NumClearedPhases++;
		cCleared = true;
	}

	mIfa_hist = mIfa;
	mIfb_hist = mIfb;
	mIfc_hist = mIfc;

	if (NumClearedPhases == 3) {
		ClearingFault = false;
	}
}

Int FaultSimulation::step(bool blocking)
{
	mSystemModel.setRightSideVectorToZero();

	for (auto eif : mExternalInterfaces) {
		eif->readValues(blocking);
	}

	for (auto comp : mComponents) {
		comp->step(mSystemModel, mTime);
	}

	mSystemModel.solve();

	for (auto comp : mComponents) {
		comp->postStep(mSystemModel);
	}

	for (auto eif : mExternalInterfaces) {
		eif->writeValues(mSystemModel);
	}

	if (ClearingFault) {
		clearFault(1, 2, 3);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);
			mComponents = mComponentsVector[++mCurrentSwitchTimeIndex];
			mLog->Log(Logger::Level::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
			mLog->Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemModel.getCurrentSystemMatrix() << std::endl;
			mLog->Log(Logger::Level::INFO) << "New decomp:" << std::endl << mSystemModel.getLUdecomp() << std::endl;
		}
	}

	mLeftVectorLog.LogNodeValues(getTime(), getLeftSideVector());
	mRightVectorLog.LogNodeValues(getTime(), getRightSideVector());

	return mTime < mFinalTime;
}
