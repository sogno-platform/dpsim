/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/SignalGenerator.h>

using namespace CPS;


Complex Signal::SignalGenerator::getSignal() {
    return mSigOut;
}

/*
Task::List Signal::SignalGenerator::getTasks() {
    return Task::List({std::make_shared<Step>(*this)});
}

void Signal::SignalGenerator::Step::execute(Real time, Int timeStepCount) {
	mSigGen.step(time);
}*/