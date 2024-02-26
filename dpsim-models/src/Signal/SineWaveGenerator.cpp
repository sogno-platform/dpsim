/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/SineWaveGenerator.h>

using namespace CPS;

Signal::SineWaveGenerator::SineWaveGenerator(String name,
                                             Logger::Level logLevel)
    : SignalGenerator(name, logLevel),
      mVoltageRef(mAttributes->createDynamic<Complex>("V_ref")) {
  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
}

void Signal::SineWaveGenerator::setParameters(Complex initialPhasor,
                                              Real frequency /*= -1*/) {
  **mVoltageRef = initialPhasor;
  mMagnitude = mVoltageRef->deriveMag();
  mPhase = mVoltageRef->derivePhase();
  **mFreq = frequency;
  **mSigOut = initialPhasor;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Parameters: \n"
                     "Sine wave magnitude: {} [V] \n"
                     "Sine wave initial phase: {} [rad] \n"
                     "Sine wave frequency: {} [Hz] \n",
                     **mMagnitude, **mPhase, **mFreq);
}

void Signal::SineWaveGenerator::step(Real time) {
  **mSigOut = Complex(**mMagnitude * cos(time * 2. * PI * **mFreq + **mPhase),
                      **mMagnitude * sin(time * 2. * PI * **mFreq + **mPhase));
}
