/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/CosineFMGenerator.h>

using namespace CPS;

void Signal::CosineFMGenerator::setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real frequency /*= 0.0*/, bool zigzag /*= false*/) {
    mMagnitude = Math::abs(initialPhasor);
    mInitialPhase = Math::phase(initialPhasor);
	mBaseFrequency = frequency;
	mModulationFrequency = modulationFrequency;
	mModulationAmplitude = modulationAmplitude;

	// default value, should implement a way to set it during runtime
	mZigZag = zigzag;

	**mSigOut = initialPhasor;
	**mFreq = frequency;

	SPDLOG_LOGGER_INFO(mSLog, "Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "\nInitial Phasor={}"
				"\nModulation Frequency={} [Hz]"
				"\nModulation Amplitude={}"
				"\nBase Frequency={} [Hz]",
				Logger::phasorToString(initialPhasor), modulationFrequency, modulationAmplitude, frequency);
}

void Signal::CosineFMGenerator::step(Real time) {
	Real phase = 2.*PI*mBaseFrequency*time + mInitialPhase;

	if(mZigZag) {
		Real tmp = 2*time*mModulationFrequency;
		Real sign = (((int)floor(tmp)) % 2 == 0) ? -1 : 1;
		phase += 2 * mModulationAmplitude * (pow(2*(tmp - floor(tmp)) - 1, 2) - 1) / PI * sign;
		**mFreq = mBaseFrequency + mModulationAmplitude * (2 * (tmp - floor(tmp)) - 1) * sign;
	} else {
		phase += mModulationAmplitude / mModulationFrequency * sin(2.*PI*mModulationFrequency*time);
		**mFreq = mBaseFrequency + mModulationAmplitude * cos(2.*PI*mModulationFrequency*time);
	}

	**mSigOut = Complex(mMagnitude * cos(phase), mMagnitude * sin(phase));
}
