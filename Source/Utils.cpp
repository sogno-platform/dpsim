/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim/MNASolverFactory.h"
#include <string>

#include <dpsim/Config.h>
#include <dpsim/Utils.h>

using namespace DPsim;
using namespace DPsim::Utils;
using namespace CPS;

#ifdef HAVE_GETOPT
  #include <getopt.h>
#else
  #include <dpsim/Compat/getopt.h>
#endif

CommandLineArgs::CommandLineArgs(int argc, char *argv[],
		String nm,
		Real dt,
		Real d,
		Real sf,
		Int s,
		CPS::Logger::Level ll,
		CPS::Logger::Level clill,
		Bool ss,
		Bool b,
		Bool si,
		CPS::Domain sd,
		Solver::Type st,
		MnaSolverFactory::MnaSolverImpl mi,
		String ps
	) :
	mProgramName(argv[0]),
	mArguments {
		{ "start-synch",	no_argument,		0, 'S', NULL, "" },
		{ "steady-init",	no_argument,		0, 'I', NULL, "" },
		{ "blocking",		no_argument,		0, 'b', NULL, "" },
		{ "help",		no_argument,		0, 'h', NULL, "" },
		{ "timestep",		required_argument,	0, 't', "SECS", "Simulation time-step" },
		{ "duration",		required_argument,	0, 'd', "SECS", "Simulation duration" },
		{ "system-freq",	required_argument,	0, 'f', "HZ", "System Frequency" },
		{ "scenario",		required_argument,	0, 's', "NUM", "Scenario selection" },
		{ "log-level",		required_argument,	0, 'l', "(NONE|INFO|DEBUG|WARN|ERR)", "Logging level" },
		{ "start-at",		required_argument,	0, 'a', "ISO8601", "Start time of real-time simulation" },
		{ "start-in",		required_argument,	0, 'i', "SECS", "" },
		{ "solver-domain",	required_argument,	0, 'D', "(SP|DP|EMT)", "Domain of solver" },
		{ "solver-type",	required_argument,	0, 'T', "(NRP|MNA)", "Type of solver" },
		{ "solver-mna-impl", required_argument, 0, 'U', "(EigenDense|EigenSparse|CUDADense|CUDASparse)", "Type of MNA Solver implementation"},
		{ "option",		required_argument,	0, 'o', "KEY=VALUE", "User-definable options" },
		{ "name",		required_argument,	0, 'n', "NAME", "Name of log files" },
		{ "params",		required_argument,	0, 'p', "PATH", "Json file containing parametrization"},
		{ 0 }
	},
	timeStep(dt),
	duration(d),
	sysFreq(sf),
	scenario(s),
	logLevel(ll),
	cliLogLevel(clill),
	name(nm),
	params(ps),
	startSynch(ss),
	blocking(b),
	steadyInit(si),
	solver{sd, st},
	mnaImpl(mi)
{
	parseArguments(argc, argv);
}

CommandLineArgs::CommandLineArgs(
		String nm,
		Real dt,
		Real d,
		Real sf,
		Int s,
		CPS::Logger::Level ll,
		CPS::Logger::Level clill,
		Bool ss,
		Bool b,
		Bool si,
		CPS::Domain sd,
		Solver::Type st,
		MnaSolverFactory::MnaSolverImpl mi
		) :
	mProgramName("dpsim"),
	mArguments {
		{ "start-synch",	no_argument,		0, 'S', NULL, "" },
		{ "steady-init",	no_argument,		0, 'I', NULL, "" },
		{ "blocking",		no_argument,		0, 'b', NULL, "" },
		{ "help",		no_argument,		0, 'h', NULL, "" },
		{ "timestep",		required_argument,	0, 't', "SECS", "Simulation time-step" },
		{ "duration",		required_argument,	0, 'd', "SECS", "Simulation duration" },
		{ "system-freq",	required_argument,	0, 'f', "HZ", "System Frequency" },
		{ "scenario",		required_argument,	0, 's', "NUM", "Scenario selection" },
		{ "log-level",		required_argument,	0, 'l', "(NONE|INFO|DEBUG|WARN|ERR)", "Logging level" },
		{ "start-at",		required_argument,	0, 'a', "ISO8601", "Start time of real-time simulation" },
		{ "start-in",		required_argument,	0, 'i', "SECS", "" },
		{ "solver-domain",	required_argument,	0, 'D', "(SP|DP|EMT)", "Domain of solver" },
		{ "solver-type",	required_argument,	0, 'T', "(NRP|MNA)", "Type of solver" },
		{ "solver-mna-impl", required_argument, 0, 'U', "(EigenDense|EigenSparse|CUDADense|CUDASparse)", "Type of MNA Solver implementation"},
		{ "option",		required_argument,	0, 'o', "KEY=VALUE", "User-definable options" },
		{ "name",		required_argument,	0, 'n', "NAME", "Name of log files" },
		{ 0 }
	},
	timeStep(dt),
	duration(d),
	sysFreq(sf),
	scenario(s),
	logLevel(ll),
	cliLogLevel(clill),
	name(nm),
	startSynch(ss),
	blocking(b),
	steadyInit(si),
	solver{sd, st},
	mnaImpl(mi)
{
}

void CommandLineArgs::parseArguments(int argc, char *argv[])
{
	mProgramName = argv[0];
	std::vector<option> long_options;
	for (auto a : mArguments)
		long_options.push_back({ a.name, a.has_arg, a.flag, a.val});

	int c;
	while (1) {
		/* getopt_long stores the option index here. */
		int option_index = 0;

		c = getopt_long(argc, argv, "ht:d:s:l:a:i:f:D:T:U:o:Sbn:", long_options.data(), &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c) {
			case 'S':
				startSynch = true;
				break;

			case 'I':
				steadyInit = true;
				break;

			case 'b':
				blocking = true;
				break;

			case 't':
				timeStep = std::stod(optarg);
				break;

			case 'd':
				duration = std::stod(optarg);
				break;

			case 'f':
				sysFreq = std::stod(optarg);
				break;

			case 's':
				scenario = std::stoi(optarg);
				break;

			case 'o': {
				String arg = optarg;

				auto p = arg.find("=");
				auto key = arg.substr(0, p);
				auto value = arg.substr(p + 1);

				if (p != String::npos) {

					// try to convert to real number
					try {
						options[key] = std::stod(value);
					}
					catch (...) {}

					// try to convert to boolean
					if (value == "true")
						options_bool[key] = true;
					else if (value == "false")
					 	options_bool[key] = false;

					// check if at least one conversion was successful
					if ((options.find(key) == options.end()) && (options_bool.find(key) == options_bool.end()))
						std::cerr << "Value " << value << " of option with key " << key << " could not be converted.";

				break;
				}
			}

			case 'l': {
				String arg = optarg;

				if (arg == "DEBUG")
					logLevel = Logger::Level::debug;
				else if (arg == "INFO")
					logLevel = Logger::Level::info;
				else if (arg == "ERR")
					logLevel = Logger::Level::err;
				else if (arg == "WARN")
					logLevel = Logger::Level::warn;
				else if (arg == "NONE")
					logLevel = Logger::Level::off;
				else
					throw std::invalid_argument("Invalid value for --log-level: must be a string of DEBUG, INFO, ERR, WARN or NONE");
				break;
			}

			case 'D': {
				String arg = optarg;

				if (arg == "DP")
					solver.domain = Domain::DP;
				else if (arg == "EMT")
					solver.domain = Domain::EMT;
				else if (arg == "SP")
					solver.domain = Domain::SP;
				else
					throw std::invalid_argument("Invalid value for --solver-domain: must be a string of SP, DP, EMT");
				break;
			}

			case 'T': {
				String arg = optarg;

				if (arg == "MNA")
					solver.type = Solver::Type::MNA;
				else if (arg == "NRP")
					solver.type = Solver::Type::NRP;
				else
					throw std::invalid_argument("Invalid value for --solver-type: must be a string of NRP or MNA");
				break;
			}
			case 'U': {
				String arg = optarg;
				if (arg == "EigenDense") {
					mnaImpl = MnaSolverFactory::EigenDense;
				} else if (arg == "EigenSparse") {
					mnaImpl = MnaSolverFactory::EigenSparse;
				} else if (arg == "CUDADense") {
					mnaImpl = MnaSolverFactory::CUDADense;
				} else if (arg == "CUDASparse") {
					mnaImpl = MnaSolverFactory::CUDASparse;
				} else if (arg == "CUDAMagma") {
					mnaImpl = MnaSolverFactory::CUDAMagma;
				} else {
					throw std::invalid_argument("Invalid value for --solver-mna-impl");
				}
				break;
			}

			case 'i': {
				double deltaT = std::stod(optarg);

				startTime = Timer::StartClock::now() + std::chrono::milliseconds(static_cast<int>(deltaT * 1e3));

				break;
			}

			case 'a': {
				std::tm t;
				std::istringstream ss(optarg);

				ss >> std::get_time(&t, "%Y%m%dT%H%M%S");

				if (ss.fail())
					throw std::invalid_argument("Invalid value for --start-at: must be a ISO8601 date");

				std::time_t tt = std::mktime(&t);

				startTime = Timer::StartClock::from_time_t(tt);

				break;
			}

			case 'n':
				name = optarg;
				break;

			case 'p':
				params = optarg;
				break;

			case 'h':
				showUsage();
				exit(0);

			case '?':
			default:
				showUsage();
				exit(-1);
		}
	}

	/* Positional arguments like files */
	while (optind < argc)
		positional.push_back(argv[optind++]);
}

void CommandLineArgs::showUsage() {
	std::cout << "Usage: " << mProgramName << " [OPTIONS] [FILES]" << std::endl;
	std::cout << std::endl;
	std::cout << " Available options:" << std::endl;

	for (auto a : mArguments) {
		if (!a.val)
			continue;

		std::cout << "  -" << static_cast<char>(a.val) << ", --" << a.name;

		if (a.valdesc)
			std::cout << " " << a.valdesc;

		if (a.desc)
			std::cout << " " << a.desc;

		std::cout << std::endl;
	}

	std::cout << std::endl;
	showCopyright();
}

void CommandLineArgs::showCopyright() {
	std::cout << "DPsim " << DPSIM_VERSION << "-" << DPSIM_RELEASE << std::endl;
	std::cout << " Copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC" << std::endl;
	std::cout << " Markus Mirz <MMirz@eonerc.rwth-aachen.de>" << std::endl;
	std::cout << " Steffen Vogel <StVogel@eonerc.rwth-aachen.de>" << std::endl;
	std::cout << " Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>" << std::endl;
}

std::list<fs::path> CommandLineArgs::positionalPaths() const {
	std::list<fs::path> paths;

	for (auto p : positional) {
		paths.emplace_back(p);
	}

	return paths;
}

String DPsim::Utils::encodeXml(String& data) {
    String buffer;
    buffer.reserve(data.size());
    for (size_t pos = 0; pos != data.size(); ++pos) {
        switch(data[pos]) {
            case '&':  buffer.append("&amp;");       break;
            case '\"': buffer.append("&quot;");      break;
            case '\'': buffer.append("&apos;");      break;
            case '<':  buffer.append("&lt;");        break;
            case '>':  buffer.append("&gt;");        break;
            default:   buffer.append(&data[pos], 1); break;
        }
    }

    return buffer;
}

std::vector<std::string> DPsim::Utils::tokenize(std::string s, char delimiter) {
	std::vector<std::string> tokens;

	size_t lastPos = 0;
	size_t curentPos;

	while ((curentPos = s.find(delimiter, lastPos)) != std::string::npos) {
		const size_t tokenLength = curentPos - lastPos;
		tokens.push_back(s.substr(lastPos, tokenLength));

		/* Advance in string */
		lastPos = curentPos + 1;
	}

	/* Check if there's a last token behind the last delimiter. */
	if (lastPos != s.length()) {
		const size_t lastTokenLength = s.length() - lastPos;
		tokens.push_back(s.substr(lastPos, lastTokenLength));
	}

	return tokens;
}

fs::path DPsim::Utils::findFile(const fs::path &name, const fs::path &hint, const std::string &useEnv) {
#ifdef _WIN32
	char sep = ';';
#else
	char sep = ':';
#endif

	std::vector<fs::path> searchPaths = {
		fs::current_path()
	};

	if (!hint.empty()) {
		searchPaths.push_back(hint);
	}

	if (!useEnv.empty() && getenv(useEnv.c_str())) {
		std::vector<std::string> envPaths = tokenize(getenv(useEnv.c_str()), sep);

		for (std::string envPath : envPaths) {
			searchPaths.emplace_back(envPath);
		}
	}

	for (auto searchPath : searchPaths) {
		fs::path fullPath;

		if (searchPath.is_relative())
			fullPath /= fs::current_path();

		fullPath /= searchPath;
		fullPath /= name;

		if (fs::exists(fullPath)) {
			return fs::absolute(fullPath);
		}
	}

	String searchPathsString;
	for (auto searchPath : searchPaths)
		searchPathsString.append(searchPath.string().append("\n"));

	throw std::runtime_error(fmt::format("File not found: {}\nSearch paths:\n{}", name.string(), searchPathsString));
}

std::list<fs::path> DPsim::Utils::findFiles(std::list<fs::path> filennames, const fs::path &hint, const std::string &useEnv) {

	std::list<fs::path> foundnames;

	for (auto filename : filennames) {
		auto foundname = findFile(filename, hint, useEnv);

		foundnames.emplace_back(foundname);
	}

	return foundnames;
}

void DPsim::Utils::applySimulationParametersFromJson(const json config, Simulation &sim){
	if (config.contains("timestep"))
			sim.setTimeStep(config["timestep"].get<double>());
	if (config.contains("duration"))
			sim.setFinalTime(config["duration"].get<double>());
}

void DPsim::Utils::applySynchronousGeneratorParametersFromJson(const json config, std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQ> syngen){
	if (config.contains("options")) {
		Bool containsSyngenOptions = false;
		for (String attrName : syngen->attrParamNames) {
			if (config["options"].contains(attrName)) {
				syngen->attribute<Real>(attrName)->set(config["options"][attrName].get<double>());
				containsSyngenOptions = true;
			}
		}
		if (containsSyngenOptions)
			syngen->applyParametersOperationalPerUnit();
	}
}
