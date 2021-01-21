/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/SineWaveGenerator.h>

using namespace CPS;

void Signal::SineWaveGenerator::setParameters(Complex initialPhasor, Real frequency /*= -1*/) {
    // mMagnitude = Math::abs(initialPhasor);
    // mInitialPhase = Math::phase(initialPhasor);
	// mFrequency = frequency;

	attribute<Complex>("sigOut")->set(initialPhasor);
	attribute<Real>("freq")->set(frequency);
}

void Signal::SineWaveGenerator::step(Real time) {
	Real freq = attribute<Real>("freq")->get();
	Real abs = Math::abs(attribute<Complex>("sigOut")->get());
	Real phase = Math::phase(attribute<Complex>("sigOut")->get());

	if (freq > 0) {
		attribute<Complex>("sigOut")->set(Complex(
			abs * cos(time * 2.*PI*freq + phase),
			abs * sin(time * 2.*PI*freq + phase)));
	}
}