/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/FrequencyRamp.h>

using namespace CPS;

void Signal::FrequencyRamp::setParameters(Complex voltageRef, Real freqStart, Real freqEnd, Real ramp, Real timeStart) {
    mVoltageRef = voltageRef;

    mFreqStart = freqStart;
    mFreqEnd = freqEnd;
    //mFreqCurr = freqStart;
    mRamp = ramp;
    mTimeStart = timeStart;
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

    attribute<Complex>("sigOut")->set(Complex(
        Math::abs(mVoltageRef) * cos(time * 2.*PI*freqCurr + Math::phase(mVoltageRef)),
        Math::abs(mVoltageRef) * sin(time * 2.*PI*freqCurr + Math::phase(mVoltageRef))));
    return attribute<Complex>("sigOut")->get();
}

Complex Signal::FrequencyRamp::getVoltage() {
    return mVoltageRef;
}
