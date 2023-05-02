/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#if defined(__clang__)
  #include <typeinfo>
#endif
#if defined(__GNUC__) || defined(__clang__)
  #include <cxxabi.h>
#endif

#include <cstdlib>
#include <list>
#include <vector>

#include <dpsim/Definitions.h>
#include <dpsim/Timer.h>
#include <dpsim/Solver.h>
#include <dpsim-models/Logger.h>
#include <dpsim/MNASolverFactory.h>
#include <dpsim-models/Filesystem.h>
#include <dpsim-models/Components.h>
#include <dpsim/Simulation.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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
		CPS::Logger::Level ll = CPS::Logger::Level::info,
		CPS::Logger::Level clill = CPS::Logger::Level::off,
		Bool ss = false,
		Bool b = false,
		Bool si = false,
		CPS::Domain sd = CPS::Domain::DP,
		Solver::Type st = Solver::Type::MNA,
		CPS::DirectLinearSolverImpl mi = CPS::DirectLinearSolverImpl::SparseLU,
		String spn = "plugin.so",
		String params = "default.json"
	);
	CommandLineArgs(
		/* Default settings */
		String name = "dpsim",
		Real dt = 0.001,
		Real d = 1,
		Real sf = 50,
		Int s = -1,
		CPS::Logger::Level ll = CPS::Logger::Level::info,
		CPS::Logger::Level clill = CPS::Logger::Level::off,
		Bool ss = false,
		Bool b = false,
		Bool si = false,
		CPS::Domain sd = CPS::Domain::DP,
		Solver::Type st = Solver::Type::MNA,
		CPS::DirectLinearSolverImpl mi = CPS::DirectLinearSolverImpl::SparseLU,
		String spn = "plugin.so"
	);

	void parseArguments(int argc, char *argv[]);
	void showUsage();
	void showCopyright();

	double timeStep;
	double duration;
	double sysFreq;
	int scenario;

	CPS::Logger::Level logLevel;
	CPS::Logger::Level cliLogLevel;
	String name;
	String params;

	bool startSynch;
	bool blocking;
	bool steadyInit;

	struct {
		CPS::Domain domain;
		Solver::Type type;
	} solver;
	CPS::DirectLinearSolverImpl directImpl;
	String solverPluginName;

	DPsim::Timer::StartClock::time_point startTime;

	std::list<String> positional;
	std::list<fs::path> positionalPaths() const;

	std::map<String, String> options;

	Int getOptionInt(String optionName) {
		// try to convert to integer number
		try{
			return std::stoi(options[optionName]);
		} catch(...) {
			throw CPS::TypeException();
		}
	}

	Real getOptionReal(String optionName) {
		// try to convert to real number
		try{
			return std::stod(options[optionName]);
		} catch(...) {
			throw CPS::TypeException();
		}
	}

	Bool getOptionBool(String optionName) {
		// try to convert to boolean
		if (options[optionName] == "true")
			return true;
		else if (options[optionName] == "false")
			return false;
		else
			throw CPS::TypeException();
	}

	String getOptionString(String optionName){
		return options[optionName];
	}

};

namespace Utils {

void applySimulationParametersFromJson(const json config, Simulation &sim);
void applySynchronousGeneratorParametersFromJson(const json config, std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQ> syngen);

String encodeXml(String& data);

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

fs::path findFile(const fs::path &name,
	const fs::path &hint = fs::path(), const std::string &useEnv = std::string());

std::list<fs::path> findFiles(std::list<fs::path> filennames,
	const fs::path &hint, const std::string &useEnv = std::string());

}
}
