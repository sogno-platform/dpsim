/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Signal/SignalGenerator.h>

namespace CPS {
namespace Signal {
	class SineWaveGenerator :
		public SignalGenerator,
        public SharedFactory<SineWaveGenerator>  {
    private:
		/// initial signal phasor with magnitude and phase
		Real mMagnitude;
		Real mInitialPhase;
    public:
		/// init the identified object
        SineWaveGenerator(String name, Logger::Level logLevel = Logger::Level::off)
			: SignalGenerator(name, logLevel) { }
		/// set the source's parameters
		void setParameters(Complex initialPhasor, Real frequency = 0.0);
		/// implementation of inherited method step to update and return the current signal value
        void step(Real time);
    };
}
}
