/** Miscelaneous utilities
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <vector>

#include <dpsim/RealTimeSimulation.h>
#include <dpsim/Solver.h>
#include <cps/Logger.h>

namespace DPsim {

class CommandLineArgs {

protected:
	struct Argument {
		char *name;
		int has_arg;
		int *flag;
		int val;
		char *valdesc;
		char *desc;
	};

	String mProgramName;
	std::vector<Argument> mArguments;

public:
	CommandLineArgs(int argc, char *argv[],
		/* Default settings */
		Real dt = 0.001,
		Real d = 1,
		Real sf = 50,
		Int s = -1,
		CPS::Logger::Level ll = CPS::Logger::Level::INFO,
		Bool ss = false,
		Bool b = false,
		CPS::Domain sd = CPS::Domain::DP,
		Solver::Type st = Solver::Type::MNA
	);

	void showUsage();
	void showCopyright();

	double timeStep;
	double duration;
	double sysFreq;
	int scenario;

	CPS::Logger::Level logLevel;

	bool startSynch;
	bool blocking;

	struct {
		CPS::Domain domain;
		Solver::Type type;
	} solver;

	DPsim::RealTimeSimulation::StartClock::time_point startTime;

	std::vector<String> positional;

	std::map<String, Real> options;
};

}
