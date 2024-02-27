/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/FrequencyRampGenerator.h>

using namespace CPS;

void Signal::FrequencyRampGenerator::setParameters(Complex initialPhasor,
                                                   Real freqStart, Real rocof,
                                                   Real timeStart,
                                                   Real duration,
                                                   bool smoothRamp) {
  mMagnitude = Math::abs(initialPhasor);
  mInitialPhase = Math::phase(initialPhasor);

  mFreqStart = freqStart;
  mRocof = rocof;
  mTimeStart = timeStart;
  mDuration = duration;
  mFreqEnd = freqStart + rocof * duration;
  mOldTime = 0.0;

  mSmoothRamp = smoothRamp;

  **mSigOut = initialPhasor;
  **mFreq = freqStart;

  SPDLOG_LOGGER_INFO(mSLog, "Parameters:");
  SPDLOG_LOGGER_INFO(mSLog,
                     "\nInitial Phasor={}"
                     "\nStart Frequency={} [Hz]"
                     "\nRoCoF={} [Hz/s]"
                     "\nStart time={} [s]"
                     "\nDuration={} [s]",
                     Logger::phasorToString(initialPhasor), freqStart, rocof,
                     timeStart, duration);
}

void Signal::FrequencyRampGenerator::step(Real time) {
  if (mUseAbsoluteCalc) {
    stepAbsolute(time);
    return;
  }

  Real currPhase;
  Real currFreq;
  Real timestep = time - mOldTime;
  mOldTime = time;

  currPhase = Math::phase(attributeTyped<Complex>("sigOut")->get());

  if (time <= mTimeStart) {
    currFreq = mFreqStart;
  } else if (time <= mTimeStart + mDuration) {
    currFreq = mFreqStart + mRocof * (time - mTimeStart);
  } else {
    currFreq = mFreqEnd;
  }
  currPhase += 2. * PI * currFreq * timestep;

  **mSigOut = mMagnitude * Complex(cos(currPhase), sin(currPhase));
  **mFreq = currFreq;
}

void Signal::FrequencyRampGenerator::stepAbsolute(Real time) {
  Real currPhase = mInitialPhase + 2 * PI * time * mFreqStart;
  Real currFreq = mFreqStart;

  if (time > mTimeStart + mDuration) {
    currPhase += 2 * PI * mRocof / 2 * pow(mDuration, 2);
    currPhase +=
        2 * PI * mRocof * mDuration * (time - (mTimeStart + mDuration));
    currFreq = mFreqEnd;
  } else if (time > mTimeStart) {
    if (mSmoothRamp) { // cos shape
      // the phase is calculated as 2pi times the integral of the frequency term below
      currPhase += 2 * PI * (mFreqEnd - mFreqStart) / 2 *
                   ((time - mTimeStart) -
                    2 * mDuration / (2 * PI) *
                        sin(2 * PI / (2 * mDuration) * (time - mTimeStart)));
      currFreq += (mFreqEnd - mFreqStart) / 2 *
                  (1 - cos(2 * PI / (2 * mDuration) * (time - mTimeStart)));
    } else { // linear
      currPhase += 2 * PI * mRocof * pow(time - mTimeStart, 2) / 2;
      currFreq += mRocof * (time - mTimeStart);
    }
  }

  **mSigOut = mMagnitude * Complex(cos(currPhase), sin(currPhase));
  **mFreq = currFreq;
}
