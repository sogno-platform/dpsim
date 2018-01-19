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

#include "SynGenSimulation.h"

using namespace DPsim;

Int SynGenSimulation::step(bool blocking)
{
	mSystemModel.setRightSideVectorToZero();
	switchSystemMatrix(mActualSystemMatrixIndex);


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

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);
			//mComponents = mComponentsVector[++mCurrentSwitchTimeIndex];
			mActualSystemMatrixIndex = mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex;
			mComponents = mComponentsVector[mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex];
			++mCurrentSwitchTimeIndex;
			mLog.Log(Logger::Level::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
			mLog.Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemModel.getCurrentSystemMatrix() << std::endl;
			mLog.Log(Logger::Level::INFO) << "New decomp:" << std::endl << mSystemModel.getLUdecomp() << std::endl;
		}
	}

	mLeftVectorLog.LogNodeValues(getTime(), getLeftSideVector());
	mRightVectorLog.LogNodeValues(getTime(), getRightSideVector());

	return mTime < mFinalTime;
}
