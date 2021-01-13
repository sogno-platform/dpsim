/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/SineWaveGenerator.h>

using namespace CPS;

void Signal::SineWaveGenerator::setParameters(Complex initialPhasor, Real frequency /*= -1*/) {
    mMagnitude = Math::abs(initialPhasor);
    mInitialPhase = Math::phase(initialPhasor);
	mFrequency = frequency;
	// initialize SignalGenerator's output value
	mSigOut = initialPhasor;
}

Complex Signal::SineWaveGenerator::step(Real time) {
	if (mFrequency < 0) {
		return mSigOut;
	} else {
		mSigOut.real(mMagnitude * cos(time * 2.*PI*mFrequency + mInitialPhase));
		mSigOut.imag(mMagnitude * sin(time * 2.*PI*mFrequency + mInitialPhase));
	}
	return mSigOut;
}