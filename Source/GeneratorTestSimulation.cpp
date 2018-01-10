/** Special simulation class for synchron generators
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

#include "GeneratorTestSimulation.h"

using namespace DPsim;

Int GeneratorTestSimulation::step(bool blocking)
{
	// Set to zero because all components will add their contribution for the current time step to the current value
	mSystemModel.getRightSideVector().setZero();

	// Execute step for all circuit components
	for (auto elm : mElements) {
		elm->step(mSystemModel, mTime);
	}

	// Solve circuit for vector j with generator output current
	mSystemModel.solve();

	// Execute PostStep for all components, generator states are recalculated based on new terminal voltage
	for (auto elm : mElements) {
		elm->postStep(mSystemModel);
	}

	if (ClearingFault) {
		clearFault(1, 2, 3);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			/*switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);
			mElements = mElementsVector[mSwitchEventVector[mCurrentSwitchTimeIndex++].systemIndex];*/
			if (mCurrentSwitchTimeIndex == 1) {
				clearFault(1, 2, 3);
				mCurrentSwitchTimeIndex++;
			}
			else {
				switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);
				mElements = mElementsVector[mSwitchEventVector[mCurrentSwitchTimeIndex++].systemIndex];
			}
			//mCurrentSwitchTimeIndex++;
			mLog->Log(Logger::Level::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
			mLog->Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemModel.getCurrentSystemMatrix() << std::endl;
			mLog->Log(Logger::Level::INFO) << "New decomp:" << std::endl << mSystemModel.getLUdecomp() << std::endl;
		}
	}

	// Save simulation step data
	if (mLastLogTimeStep == 0) {
		mLeftVectorLog->LogNodeValues(getTime(), getLeftSideVector());
		mRightVectorLog->LogNodeValues(getTime(), getRightSideVector());
	}

	mLastLogTimeStep++;
	if (mLastLogTimeStep == mDownSampleRate) {
		mLastLogTimeStep = 0;
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
}
