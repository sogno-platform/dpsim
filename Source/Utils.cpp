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

#include <getopt.h>
#include <string>
#include <chrono>

#include "Utils.h"

using namespace DPsim;

DPsim::CommandLineArgs::CommandLineArgs(int argc, char *argv[]) :
	mProgramName(argv[0]),
	mArguments {
		{ "start-synch",	no_argument,		0, 'S', NULL, "" },
		{ "blocking",		no_argument,		0, 'b', NULL, "" },
		{ "help",		no_argument,		0, 'h', NULL, "" },
		{ "timestep",		required_argument,	0, 't', "SECS", "" },
		{ "duration",		required_argument,	0, 'd', "SECS", "" },
		{ "scenario",		required_argument,	0, 's', "NUM", "" },
		{ "log-level",		required_argument,	0, 'l', "(NONE|INFO|DEBUG|WARN|ERROR)", "" },
		{ "start-at",		required_argument,	0, 'a', "ISO8601", "" },
		{ "start-in",		required_argument,	0, 'i', "SECS", "" },
		{ "solver-domain",	required_argument,	0, 'D', "(DP|EMT)", "" },
		{ "solver-type",	required_argument,	0, 'T', "(MNA)", "" },
		{ "option",		required_argument,	0, 'o', "KEY=VALUE", "" },
		{ 0 }
	}
{
	/* Default settings */
	timeStep = 0.001;
	duration = 1;
	logLevel = Logger::Level::INFO;

	startSynch = false;
	blocking = false;

	solver.domain = Solver::Domain::DP;
	solver.type = Solver::Type::MNA;

	std::vector<option> long_options;
	for (auto a : mArguments)
		long_options.push_back({ a.name, a.has_arg, a.flag, a.val});

	int c;
	while (1) {
		/* getopt_long stores the option index here. */
		int option_index = 0;

		c = getopt_long(argc, argv, "ht:d:s:l:a:i:D:T:o:Sb", long_options.data(), &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c) {
			case 'S':
				startSynch = true;
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

			case 's':
				scenario = std::stoi(optarg);
				break;

			case 'o': {
				String arg = optarg;
				String key;
				Real value;
				int p = arg.find("=");
				key = arg.substr(0, p);
				if (p != String::npos)
					value = std::stod(arg.substr(p + 1));

				options[key] = value;
				break;
			}

			case 'l': {
				String arg = optarg;

				if (arg == "DEBUG")
					logLevel = Logger::Level::DEBUG;
				else if (arg == "INFO")
					logLevel = Logger::Level::INFO;
				else if (arg == "ERROR")
					logLevel = Logger::Level::ERROR;
				else if (arg == "WARN")
					logLevel = Logger::Level::WARN;
				else if (arg == "NONE")
					logLevel = Logger::Level::NONE;
				else
					throw std::invalid_argument("Invalid value for --log-level: must be a string of DEBUG, INFO, ERROR, WARN or NONE");
				break;
			}

			case 'D': {
				String arg = optarg;

				if (arg == "DP")
					solver.domain = Solver::Domain::DP;
				else if (arg == "EMT")
					solver.domain = Solver::Domain::EMT;
				else
					throw std::invalid_argument("Invalid value for --solver-domain: must be a string of DP, EMT");
				break;
			}

			case 'T': {
				String arg = optarg;

				if (arg == "MNA")
					solver.type = Solver::Type::MNA;
				else
					throw std::invalid_argument("Invalid value for --solver-type: must be a string of MNA");
				break;
			}

			case 'i': {
				double deltaT = std::stod(optarg);

				startTime = RealTimeSimulation::StartClock::now() + std::chrono::milliseconds(static_cast<int>(deltaT * 1e3));

				break;
			}

			case 'a': {
				std::tm t;
				std::istringstream ss(optarg);

				ss >> std::get_time(&t, "%Y%m%dT%H%M%S");

				if (ss.fail())
					throw std::invalid_argument("Invalid value for --start-at: must be a ISO8601 date");

				std::time_t tt = std::mktime(&t);

				startTime = RealTimeSimulation::StartClock::from_time_t(tt);

				break;
			}

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

void DPsim::CommandLineArgs::showUsage() {
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

void DPsim::CommandLineArgs::showCopyright() {
	std::cout << "DPsim " << DPSIM_VERSION << "-" << DPSIM_RELEASE << std::endl;
	std::cout << " Copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC" << std::endl;
	std::cout << " Markus Mirz <MMirz@eonerc.rwth-aachen.de>" << std::endl;
	std::cout << " Steffen Vogel <StVogel@eonerc.rwth-aachen.de>" << std::endl;
}
