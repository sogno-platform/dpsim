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

    mSigOut = initialPhasor;
}

Complex Signal::FrequencyRamp::step(Real time) {
    Real freqCurr;
    /// TODO: calculate signal value and set mSigOut attribute 
    if(time > mTimeStart) {
        freqCurr = mFreqStart + mRamp * (time - mTimeStart);
        if(freqCurr > mFreqEnd) freqCurr = mFreqEnd;
    } else {
        freqCurr = mFreqStart;
    }

    mSigOut.real(mMagnitude * cos(time * 2.*PI*freqCurr + mInitialPhase));
    mSigOut.imag(mMagnitude * sin(time * 2.*PI*freqCurr + mInitialPhase));

    return mSigOut;
}