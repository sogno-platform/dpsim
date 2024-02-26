/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <fstream>

#include <dpsim-models/Signal/TurbineGovernor.h>

using namespace CPS;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {
  // Define Object for saving data on a file
  std::ofstream log("TurbineOutput.csv");
  std::ifstream omega("omega.csv");

  // Define machine parameters in per unit
  Real nomPower = 555e6;

  // Turbine
  Real Ta_t = 0.3;
  Real Fa = 0.3;
  Real Tb = 7;
  Real Fb = 0.3;
  Real Tc = 0.2;
  Real Fc = 0.4;
  Real Tsr = 0.1;
  Real Tsm = 0.3;
  Real Kg = 20;

  Real initActivePower = 300e6;

  Signal::TurbineGovernor governor("governor");
  governor.setParameters(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm);

  Real OmRef = 1;
  Real PmRef = 300e6 / 555e6;

  std::string line;
  Real Om;
  Real dt = 0.00005;
  Real t = 0;
  Real Pm = PmRef;

  governor.initialize(PmRef, initActivePower / nomPower);

  while (getline(omega, line)) {
    t = t + dt;
    Om = std::stod(line);
    std::cout << Om << '\n';
    Pm = governor.step(Om, OmRef, PmRef, dt);
    log << t << "," << Pm << std::endl;
  }

  return 0;
}
