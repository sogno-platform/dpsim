#include <iostream>
#include <string>
#include "CIMReader.h"
#include "Simulation.h"
#include "ShmemInterface.h"

using namespace DPsim;

void usage() {
	std::cerr << "usage: DPsim [OPTIONS] CIM_FILE..." << std::endl
	  << "Possible options:" << std::endl
	  << "  -d/--duration DURATION:   simulation duration in seconds (default: 0.3)" << std::endl
	  << "  -f/--frequency FREQUENCY: system frequency in Hz (default: 50)" << std::endl
	  << "  -h/--help:                show this help and exit" << std::endl
	  << "  -i/--interface OBJ_NAME:  prefix for the names of the shmem objects used for communication (default: /dpsim)" << std::endl
	  << "  -n/--node NODE_ID:        RDF id of the node where the interfacing voltage/current source should be placed" << std::endl
	  << "  -s/--split INDEX:         index of this instance for distributed simulation (0 or 1)" << std::endl
	  << "  -t/--timestep TIMESTEP:   simulation timestep in seconds (default: 1e-3)" << std::endl;
}

bool parseFloat(const char *s, double *d) {
	char *end;
	*d = std::strtod(s, &end);
	return (end != s && !*end);
}

bool parseInt(const char *s, int *i) {
	char *end;
	*i = strtol(s, &end, 0);
	return (end != s && !*end);
}

int main(int argc, const char* argv[]) {
	int i, split = -1;
	Real frequency = 2*PI*50, timestep = 0.001, duration = 0.3;
	std::string interfaceBase = "/dpsim";
	std::string splitNode = "";
	std::string outName, inName, logName, llogName, rlogName;
	ShmemInterface *intf = nullptr;

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
		} else if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--interface")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -i/--interface; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (argv[++i][0] != '/') {
				std::cerr << "Shmem interface object name must start with a '/'" << std::endl;
				return 1;
			}
			interfaceBase = std::string(argv[i]);
		} else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--split")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -s/--split; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			if (!parseInt(argv[++i], &split) || split < 0 || split > 1) {
				std::cerr << "Invalid setting " << argv[i] << " for the split index" << std::endl;
				return 1;
			}
		} else if (!strcmp(argv[i], "-n") || !strcmp(argv[i], "--node")) {
			if (i == argc-1) {
				std::cerr << "Missing argument for -n/--node; see 'DPsim --help' for usage" << std::endl;
				return 1;
			}
			splitNode = std::string(argv[++i]);
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
	reader.parseFiles();
	std::vector<BaseComponent*> components = reader.getComponents();

	// TODO: this is a simple, pretty much fixed setup. Make this more flexible / configurable
	if (split >= 0) {
		int node = reader.mapTopologicalNode(splitNode);
		if (node < 0) {
			std::cerr << "Invalid / missing split node" << std::endl;
			return 1;
		}
		if (split == 0) {
			outName = interfaceBase + ".0.out";
			inName = interfaceBase + ".0.in";
			intf = new ShmemInterface(outName.c_str(), inName.c_str());
			ExternalVoltageSource *evs = new ExternalVoltageSource("v_int", node, 0, 0, 0, reader.getNumVoltageSources()+1);
			intf->registerVoltageSource(evs, 0, 1);
			intf->registerExportedCurrent(evs, 0, 1);
			components.push_back(evs);
			// TODO make log names configurable
			logName = "cim0.log";
			llogName = "lvector-cim0.csv";
			rlogName = "rvector-cim0.csv";
		} else {
			outName = interfaceBase + ".1.out";
			inName = interfaceBase + ".1.in";
			intf = new ShmemInterface(outName.c_str(), inName.c_str());
			ExternalCurrentSource *ecs = new ExternalCurrentSource("i_int", node, 0, 0, 0);
			intf->registerCurrentSource(ecs, 0, 1);
			intf->registerExportedVoltage(node, 0, 0, 1);
			components.push_back(ecs);
			logName = "cim1.log";
			llogName = "lvector-cim1.csv";
			rlogName = "rvector-cim1.csv";
		}
	}

	// Do the actual simulation
	Logger log(logName), llog(llogName), rlog(rlogName);
	Simulation sim(components, frequency, timestep, duration, log);
	if (intf)
		sim.addExternalInterface(intf);
	sim.runRT(RTTimerFD, true, log, llog, rlog);
	if (intf)
		delete intf;
	return 0;
}
