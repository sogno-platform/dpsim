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

DPsim::CommandLineArgs::CommandLineArgs(int argc, char *argv[]) {

	int c;

	while (1) {
		static struct option long_options[] = {
			{ "timestep",	required_argument,	0, 't' },
			{ "duration",	required_argument,	0, 'd' },
			{ "scenario",	required_argument,	0, 's' },
			{ "log-level",	required_argument,	0, 'l' },
			{ "start-at",	required_argument,	0, 'a' },
			{ "start-in",	required_argument,	0, 'i' },
			{ 0, 0, 0, 0 }
		};

		/* getopt_long stores the option index here. */
		int option_index = 0;

		c = getopt_long(argc, argv, "t:d:s:l:a:i:", long_options, &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c) {
			/* If this option set a flag, do nothing else now. */
			case 0:
				if (long_options[option_index].flag != 0)
					break;

#if 0
				printf ("option %s", long_options[option_index].name);

				if (optarg)
					printf (" with arg %s", optarg);
				printf ("\n");
#endif
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

			case 'l':
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
					throw std::invalid_argument();
				break;

			case 'i':
				double deltaT = std::stod(optarg);

				startTime = RealTimeSimulation::StartClock::now() + std::chrono::milliseconds(deltaT * 1e3);

				break;

			case 'a': {
				std::tm t;
				std::istringstream ss(optarg);

				ss >> std::get_time(&t, "%Y%m%dT%H%M%S");

				if (ss.fail())
					throw std::invalid_argument();

				std::time_t tt = std::mktime(&t);

				startTime = RealTimeSimulation::StartClock::from_time_t(tt);

				break;
			}

			case '?':
				/* getopt_long already printed an error message. */
				break;

			default:
				abort ();
		}
	}

	/* Positional arguments like files */
	while (optind < argc)
		positional.push_back(argv[optind++])
}
