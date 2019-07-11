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

#if defined(__GNUC__) && !defined(__clang__)
  #include <cxxabi.h>
#endif

#include <cstdlib>
#include <list>
#include <vector>
#include <experimental/filesystem>

#include <dpsim/Timer.h>
#include <dpsim/Solver.h>
#include <cps/Logger.h>

namespace fs = std::experimental::filesystem;

namespace DPsim {

class CommandLineArgs {

protected:
	struct Argument {
		const char *name;
		int has_arg;
		int *flag;
		int val;
		const char *valdesc;
		const char *desc;
	};

	String mProgramName;
	std::vector<Argument> mArguments;

public:
	CommandLineArgs(int argc, char *argv[],
		/* Default settings */
		String name = "dpsim",
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
	String name;

	bool startSynch;
	bool blocking;

	struct {
		CPS::Domain domain;
		Solver::Type type;
	} solver;

	DPsim::Timer::StartClock::time_point startTime;

	std::list<String> positional;

	std::map<String, Real> options;
};

namespace Utils {

template<typename T>
static CPS::String type(const CPS::String &stripPrefix = "CPS::") {
	Int status = 1;
	const char *mangled, *unmangled;

	mangled = typeid(T).name();

#ifdef _MSC_VER
	return CPS::String(mangled);
#else
	unmangled = abi::__cxa_demangle(mangled, NULL, NULL, &status);

	if (status)
		return mangled;
	else {
		CPS::String type = unmangled;

		delete unmangled;

		if (type.find(stripPrefix) == 0)
			type = type.substr(stripPrefix.size());

		return type;
	}
#endif
}

std::vector<std::string> tokenize(std::string s, char delimiter);

fs::path findFile(const fs::path &name, const fs::path &hint = fs::path(), const std::string &useEnv = std::string());

std::list<fs::path> findFiles(std::list<fs::path> filennames, const fs::path &hint, const std::string &useEnv = std::string());

}
}
