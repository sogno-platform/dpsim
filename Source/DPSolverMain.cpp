#include <iostream>
#include <string>
#include "MathLibrary.h"
#include "LinearDPSim.h"
#include "Components.h"
#include "Log.h"
#include "TopologyReader.h"

void readCmdLineArguments(char* &confFilename, int argc, char* argv[]);

int main(int argc, char* argv[]) {

	// Extract command line arguments
	char* confFilename = nullptr;	
	readCmdLineArguments(confFilename, argc, argv);	

	// Read config file
	std::ifstream confFile;
	Config conf;
	TopologyReader topoReader;
	confFile.open(confFilename);
	if (topoReader.readConfig(confFile, conf) != 0 || conf.elements.size() == 0) {
		std::cerr << "netlist file or path invalid" << std::endl;
		exit(1);
	}		
		
	// Parse config file
	double tf, dt, t;
	double om = 2.0*M_PI*50.0;
	std::vector<CircuitElement*> circElements;
	topoReader.parseConfig(conf, circElements, dt, tf);
	if (circElements.size() == 0) {
		std::cerr << "failed to parse netlist" << std::endl;
		exit(1);
	}

	// Define Object for saving data on a file
	Log log("data.csv", "log.txt");
	
	// Add components to new simulation	and create system matrix
	std::cout << "Create System matrix" << std::endl;

	LinearDPSim newSim(circElements, om, dt, tf);	

	std::cout << "Init Completed" << std::endl;

	std::cout << "Entering Main Simulation Loop" << std::endl;

	// Get Current Simulation Time
	t = newSim.getTime();

	// Main Simulation Loop
	while (newSim.step())
	{
		// Save Simulation Step
		log.AddDataLine(t, newSim.getVoltages());
		// Get Current Simulation Time
		t = newSim.getTime();
	}
	
	std::cout << "Simulation finished" << std::endl;
	std::cin.get();
	return 0;
}

void readCmdLineArguments(char* &confFilename, int argc, char* argv[]) {
	for (int i = 1; i < argc; i++) {
		// Find and extract netlist file path
		if (!std::string(argv[i]).compare("-nl") || !std::string(argv[i]).compare("--netlist")) {
			if (++i >= argc) {
				std::cerr << "missing parameter for -nl/--netlist" << std::endl;
				exit(1);
			}
			confFilename = argv[i];
		}
		else {
			std::cerr << "unknown / invalid parameter " << argv[i] << std::endl;
			exit(1);
		}
	}

	if (!confFilename) {
		std::cerr << "no netlist file given" << std::endl;
		exit(1);
	}
}