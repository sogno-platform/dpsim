/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/FIRFilter.h>
#include <iostream>

using namespace CPS;
using namespace CPS;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {
  std::vector<Real> coefficients = {
      -0.0024229, -0.0020832, 0.0067703, 0.016732,  0.011117,  -0.0062311,
      -0.0084016, 0.0092568,  0.012983,  -0.010121, -0.018274, 0.011432,
      0.026176,   -0.012489,  -0.037997, 0.013389,  0.058155,  -0.014048,
      -0.10272,   0.014462,   0.31717,   0.48539,   0.31717,   0.014462,
      -0.10272,   -0.014048,  0.058155,  0.013389,  -0.037997, -0.012489,
      0.026176,   0.011432,   -0.018274, -0.010121, 0.012983,  0.0092568,
      -0.0084016, -0.0062311, 0.011117,  0.016732,  0.0067703, -0.0020832,
      -0.0024229};

  FIRFilter filter("filter", coefficients, 1);
  Attribute<Real>::Ptr inputAttr = AttributeStatic<Real>::make(10);

  filter.initialize(1);
  filter.setInput(inputAttr);

  for (int i = 0; i < 1000; i++) {
    if (i == 500)
      **inputAttr = 5;

    filter.step(i);
  }
}
