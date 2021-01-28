/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/FrequencyRamp.h>

using namespace CPS;

void Signal::FrequencyRamp::setParameters(Complex initialPhasor, Real freqStart, Real ramp, Real timeStart, Real duration, bool useAbsoluteCalc) {
    mMagnitude = Math::abs(initialPhasor);
    mInitialPhase = Math::phase(initialPhasor);

    mFreqStart = freqStart;
    mRamp = ramp;
    mTimeStart = timeStart;
    mDuration = duration;
    mFreqEnd = freqStart + ramp * duration;
    mOldTime = 0.0;

    mUseAbsoluteCalc = useAbsoluteCalc;

    attribute<Complex>("sigOut")->set(initialPhasor);
	attribute<Real>("freq")->set(freqStart);
}

void Signal::FrequencyRamp::step(Real time) {
    if(mUseAbsoluteCalc) {
        stepAbsolute(time);
        return;
    }

    Real currPhase;
    Real currFreq;
    Real timestep = time - mOldTime;
    mOldTime = time;
    
    currPhase = Math::phase(attribute<Complex>("sigOut")->get());

    if(time <= mTimeStart) {
        currFreq = mFreqStart;
    } else if(time <= mTimeStart + mDuration) {
        currFreq = mFreqStart + mRamp * (time - mTimeStart);
    } else {
        currFreq = mFreqEnd;
    }
    currPhase += 2. * PI * currFreq * timestep;

    attribute<Complex>("sigOut")->set(mMagnitude * Complex(cos(currPhase), sin(currPhase)));
    attribute<Real>("freq")->set(currFreq);
}

void Signal::FrequencyRamp::stepAbsolute(Real time) {
    Real currPhase = mInitialPhase;
    Real currFreq = mFreqStart;

    if(time <= mTimeStart) {
        currPhase += 2 * PI * time * mFreqStart;
    } else {
        currPhase += 2 * PI * mTimeStart * mFreqStart;

        if(time <= mTimeStart + mDuration) {
            currPhase += 2 * PI * mRamp / 2 * pow(time - mTimeStart, 2);
            currFreq += mRamp * (time - mTimeStart);
        } else {
            currPhase += 2 * PI * mRamp / 2 * pow(mDuration, 2);
            currPhase += 2 * PI * mFreqEnd * (time - (mTimeStart + mDuration));
            currFreq = mFreqEnd;
        }
    }

    attribute<Complex>("sigOut")->set(mMagnitude * Complex(cos(currPhase), sin(currPhase)));
    attribute<Real>("freq")->set(currFreq);
}