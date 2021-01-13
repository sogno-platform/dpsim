/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/SineWaveGenerator.h>

using namespace CPS;

Signal::SineWaveGenerator::SineWaveGenerator(String name, Logger::Level logLevel /*= Logger::Level::off*/)
    : Signal::SignalGenerator(name, logLevel) {

	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

Complex Signal::SineWaveGenerator::step(Real time) {
	if (attribute<Real>("f_src")->get() < 0) {
		attribute<Complex>("sigOut")->set(attribute<Complex>("V_ref")->get());
	}
	else {
		attribute<Complex>("sigOut")->set(Complex(
				Math::abs(attribute<Complex>("V_ref")->get()) * cos(time * 2.*PI*attribute<Real>("f_src")->get() + Math::phase(attribute<Complex>("V_ref")->get())),
				Math::abs(attribute<Complex>("V_ref")->get()) * sin(time * 2.*PI*attribute<Real>("f_src")->get() + Math::phase(attribute<Complex>("V_ref")->get()))));
	}
	return attribute<Complex>("sigOut")->get();
}

Complex Signal::SineWaveGenerator::getVoltage() {
	return attribute<Complex>("V_ref")->get();
}

void Signal::SineWaveGenerator::setParameters(Complex voltageRef, Real srcFreq /*= -1*/) {
    attribute<Complex>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);
}

