/* Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2023-2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: MPL-2.0
 */

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
