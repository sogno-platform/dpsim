/* Copyright 2024 Institute for Automation of Complex Power Systems,
 *                EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/DCGenerator.h>

using namespace CPS;

Signal::DCGenerator::DCGenerator(String name, Logger::Level logLevel)
    : SignalGenerator(name, logLevel),
      mVoltageRef(mAttributes->createDynamic<Complex>("V_ref")) {
  **mFreq = 0.0;
  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
}

void Signal::DCGenerator::setParameters(Real initialReference) {
  **mVoltageRef = initialReference;
  **mSigOut = initialReference;
}

void Signal::DCGenerator::step(Real time) { **mSigOut = **mVoltageRef; }
