/** Utilities
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
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

#pragma once

#include <string>
#include "Definitions.h"

namespace DPsim {

	void updateProgressBar(Real time, Real finalTime);
	void usage();
	bool parseFloat(const char *s, double *d);
	bool parseInt(const char *s, int *i);
	Int parseArguments(int argc, const char* argv[], bool &rt, bool &batch, Int &split, String &interfaceBase, String &splitNode);
	Int checkArguments(bool rt, Int split, String splitNode);
	Int consoleMain(int argc, const char* argv[]);
}

