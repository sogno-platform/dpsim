/** Utilities
 *
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

#include "Utilities.h"

#include <iostream>

using namespace DPsim;

void DPsim::updateProgressBar(Real time, Real finalTime) {

	if (time / finalTime <= 0.1) {
		std::cout << "                      (0%)\r";
	}
	else if (time / finalTime > 0.1 && time / finalTime <= 0.2) {
		std::cout << "##                    (10%)\r";
	}
	else if (time / finalTime > 0.2 && time / finalTime <= 0.3) {
		std::cout << "####                  (20%)\r";
	}
	else if (time / finalTime > 0.3 && time / finalTime <= 0.4) {
		std::cout << "######                (30%)\r";
	}
	else if (time / finalTime > 0.4 && time / finalTime <= 0.5) {
		std::cout << "########              (40%)\r";
	}
	else if (time / finalTime > 0.5 && time / finalTime <= 0.6) {
		std::cout << "##########            (50%)\r";
	}
	else if (time / finalTime > 0.6 && time / finalTime <= 0.7) {
		std::cout << "############          (60%)\r";
	}
	else if (time / finalTime > 0.7 && time / finalTime <= 0.8) {
		std::cout << "##############        (70%)\r";
	}
	else if (time / finalTime > 0.8 && time / finalTime <= 0.9) {
		std::cout << "################      (80%)\r";
	}
	else if (time / finalTime > 0.9 && time / finalTime < 1) {
		std::cout << "##################    (90%)\r";
	}
	else {
		std::cout << "####################  (100%)";
		std::cout << std::endl;
	}
}

void DPsim::usage() {
	std::cerr << "usage: DPsolver [OPTIONS] [PYTHON_FILE...]" << std::endl
		<< "Possible options:" << std::endl
		<< "  -b/--batch:               don't show an interactive prompt after reading files" << std::endl
		<< "  -h/--help:                show this help and exit" << std::endl
		<< "  -i/--interface OBJ_NAME:  prefix for the names of the shmem objects used for communication (default: /dpsim)" << std::endl
		<< "  -n/--node NODE_ID:        RDF id of the node where the interfacing voltage/current source should be placed" << std::endl
		<< "  -r/--realtime:            enable realtime simulation " << std::endl
		<< "  -s/--split INDEX:         index of this instance for distributed simulation (0 or 1)" << std::endl
		<< "Remaining arguments are treated as Python files and executed in order." << std::endl;
}

bool DPsim::parseFloat(const char *s, double *d) {
	char *end;
	*d = std::strtod(s, &end);
	return (end != s && !*end);
}

bool DPsim::parseInt(const char *s, int *i) {
	char *end;
	*i = strtol(s, &end, 0);
	return (end != s && !*end);
}


int DPsim::parseArguments(int argc, const char* argv[],
	bool &rt, bool &batch, Int &split, String &interfaceBase, String &splitNode) {

	Int i;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-b") || !strcmp(argv[i], "--batch")) {
			batch = true;
		}
		else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			usage();
			return 0;
		}
		else if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--interface")) {
			if (i == argc - 1) {
				std::cerr << "Missing argument for -i/--interface; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (argv[++i][0] != '/') {
				std::cerr << "Shmem interface object name must start with a '/'" << std::endl;
				return 1;
			}
			interfaceBase = String(argv[i]);
		}
		else if (!strcmp(argv[i], "-n") || !strcmp(argv[i], "--node")) {
			if (i == argc - 1) {
				std::cerr << "Missing argument for -n/--node; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			splitNode = String(argv[++i]);
		}
		else if (!strcmp(argv[i], "-r") || !strcmp(argv[i], "--realtime")) {
			rt = true;
		}
		else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--split")) {
			if (i == argc - 1) {
				std::cerr << "Missing argument for -s/--split; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseInt(argv[++i], &split) || split < 0 || split > 1) {
				std::cerr << "Invalid setting " << argv[i] << " for the split index" << std::endl;
				return 1;
			}
		}
		else if (argv[i][0] == '-') {
			std::cerr << "Unknown option " << argv[i] << " ; see 'DPsim --help' for usage" << std::endl;
			return 1;
		}
		else {
			// remaining arguments treated as input files
			break;
		}
	}
}

int DPsim::checkArguments(bool rt, Int split, String splitNode) {
#ifndef __linux__
	if (split >= 0 || splitNode.length() != 0) {
		std::cerr << "Distributed simulation not supported on this platform" << std::endl;
		return 1;
	}
	else if (rt) {
		std::cerr << "Realtime simulation not supported on this platform" << std::endl;
		return 1;
	}
#endif
	if (split >= 0 || splitNode.length() != 0 || rt) {
		std::cerr << "Realtime and distributed simulation currently not supported in combination with Python" << std::endl;
		return 1;
	}
}

// Console main that supports command line arguments
int DPsim::consoleMain(int argc, const char* argv[]) {
	bool rt = false;
	bool batch = false;
	int split = -1;
	String interfaceBase = "/dpsim";
	String splitNode = "";
	String outName, inName, logName("log.txt"), llogName("lvector.csv"), rlogName("rvector.csv");

	if (parseArguments(argc, argv, rt, batch, split, interfaceBase, splitNode) == 1) {
		return 1;
	}

	if (checkArguments(rt, split, splitNode) == 1) {
		return 1;
	}
	return 0;
}