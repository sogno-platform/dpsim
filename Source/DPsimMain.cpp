#include <iostream>
#include <string>
#include "CIMReader.h"
#include "Simulation.h"

using namespace DPsim;

void usage() {
	std::cerr << "usage: DPsim [OPTIONS] CIM_FILE..." << std::endl
	  << "Possible options:" << std::endl
	  << "  -d/--duration DURATION:   simulation duration in seconds (default: 0.3)" << std::endl
	  << "  -h/--help:                show this help and exit" << std::endl
	  << "  -f/--frequency FREQUENCY: system frequency in Hz (default: 50)" << std::endl
	  << "  -t/--timestep TIMESTEP:   simulation timestep in seconds (default: 1e-3)" << std::endl;
}

bool parseFloat(const char *s, double *d) {
	char *end;
	*d = std::strtod(s, &end);
	return (end != s && !*end);
}

int main(int argc, const char* argv[]) {
	Real frequency = 2*PI*50, timestep = 0.001, duration = 0.3;
	int i;

	// Parse arguments
	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--frequency")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -f/--frequency; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseFloat(argv[++i], &frequency) || frequency <= 0) {
				std::cerr << "Invalid setting " << argv[i] << " for system frequency" << std::endl;
				return 1;
			}
			frequency *= 2*PI;
		} else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--timestep")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -t/--timestep; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseFloat(argv[++i], &timestep) || timestep <= 0) {
				std::cerr << "Invalied setting " << argv[i] << " for the timestep" << std::endl;
				return 1;
			}
		} else if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--duration")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -d/--duration; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseFloat(argv[++i], &duration) || duration <= 0) {
				std::cerr << "Invalid setting " << argv[i] << " for the duration" << std::endl;
				return 1;
			}
		} else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			usage();
			return 0;
		} else if (argv[i][0] == '-') {
			std::cerr << "Unknown option " << argv[i] << " ; see 'DPsim --help' for usage" << std::endl;
			return 1;
		} else {
			// remaining arguments treated as input files
			break;
		}
	}
	if (i == argc) {
		std::cerr << "No input files given (see DPsim --help for usage)" << std::endl;
		return 1;
	}

	// Parse CIM files
	CIMReader reader(frequency);
	for (; i < argc; i++) {
		if (!reader.addFile(argv[i])) {
			std::cerr << "Failed to read file " << argv[i] << std::endl;
			return 1;
		}
	}
	std::vector<BaseComponent*> components = reader.mapComponents();

	// Do the actual simulation
	Logger log("cim.log"), llog("lvector-cim.csv"), rlog("rvector-cim.csv");
	Simulation sim(components, frequency, timestep, duration, log);
	while (sim.step(log, llog, rlog))
		sim.increaseByTimeStep();
	return 0;
}

