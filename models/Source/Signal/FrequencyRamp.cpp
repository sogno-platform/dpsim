/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/FrequencyRamp.h>

using namespace CPS;

void Signal::FrequencyRamp::setParameters(Complex initialPhasor, Real freqStart, Real freqEnd, Real ramp, Real timeStart) {
    mMagnitude = Math::abs(initialPhasor);
    mInitialPhase = Math::phase(initialPhasor);

    mFreqStart = freqStart;
    mFreqEnd = freqEnd;
    mRamp = ramp;
    mTimeStart = timeStart;

    attribute<Complex>("sigOut")->set(initialPhasor);
	attribute<Real>("freq")->set(freqStart);
}

void Signal::FrequencyRamp::step(Real time) {
    Real freq;
	//Real abs = Math::abs(attribute<Complex>("sigOut")->get());
	//Real phase = Math::phase(attribute<Complex>("sigOut")->get());
    
    /// TODO: calculate signal value and set mSigOut attribute 
    if(time > mTimeStart) {
        freq = mFreqStart + mRamp * (time - mTimeStart);
        if((freq < mFreqEnd) == (mRamp < 0)) freq = mFreqEnd;
    } else {
        freq = mFreqStart;
    }

    if(time > 2) {
        freq = freq;
    }

    attribute<Complex>("sigOut")->set(Complex(
        mMagnitude * cos(time * 2.*PI*freq + mInitialPhase),
        mMagnitude * sin(time * 2.*PI*freq + mInitialPhase)));
    attribute<Real>("freq")->set(freq);
}