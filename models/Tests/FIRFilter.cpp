/**
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
*
* CPowerSystems
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************************/

#include <iostream>
#include <cps/Signal/FIRFilter.h>

using namespace CPS;
using namespace CPS;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {
	std::vector<Real> coefficients = {
		-0.0024229,-0.0020832,0.0067703,0.016732,0.011117,-0.0062311,-0.0084016,0.0092568,
		0.012983,-0.010121,-0.018274,0.011432,0.026176,-0.012489,-0.037997,0.013389,0.058155,-0.014048,
		-0.10272,0.014462,0.31717,0.48539, 0.31717,0.014462,-0.10272,-0.014048,0.058155,0.013389,-0.037997,
		-0.012489,0.026176,0.011432,-0.018274,-0.010121, 0.012983,0.0092568,-0.0084016,-0.0062311,0.011117,
		0.016732,0.0067703,-0.0020832,-0.0024229
	};

	FIRFilter filter("filter", coefficients, 1);
	Real input = 10;
	Attribute<Real>::Ptr inputAttr = Attribute<Real>::make(&input);

	filter.initialize(1);
	filter.setInput(inputAttr);

	for (int i = 0; i < 1000; i++) {
		if (i == 500)
			input = 5;

		filter.step(i);
	}
}
