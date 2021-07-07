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
	class CosineFMGenerator :
		public SignalGenerator,
        public SharedFactory<CosineFMGenerator>  {
    private:
		/// initial signal phasor with magnitude and phase
		Real mMagnitude;
		Real mInitialPhase;
		/// signal's frequency
		Real mBaseFrequency;
		/// Modulation parameters
		Real mModulationFrequency;
		Real mModulationAmplitude;
		/// toggle a zig zag like frequency modulation
		bool mZigZag = false;

    public:
		/// init the identified object
        CosineFMGenerator(String name, Logger::Level logLevel = Logger::Level::off)
			: SignalGenerator(name, logLevel) { }
		/// set the source's parameters
		void setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real frequency = 0.0, bool zigzag = false);
		/// implementation of inherited method step to update and return the current signal value
        void step(Real time);
    };
}
}
