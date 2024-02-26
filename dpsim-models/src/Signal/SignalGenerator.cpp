/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/SignalGenerator.h>

using namespace CPS;

Signal::SignalGenerator::SignalGenerator(String uid, String name,
                                         Logger::Level logLevel)
    : SimSignalComp(name, logLevel),
      mSigOut(mAttributes->create<Complex>("sigOut")),
      mFreq(mAttributes->createDynamic<Real>("freq")) {

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
}

Complex Signal::SignalGenerator::getSignal() { return **mSigOut; }
