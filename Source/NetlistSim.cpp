#include "NetlistSim.h"

void NetlistSim(int argc, char* argv[]) {

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
	std::vector<BaseComponent*> circElements;
	topoReader.parseConfig(conf, circElements, dt, tf);
	if (circElements.size() == 0) {
		std::cerr << "failed to parse netlist" << std::endl;
		exit(1);
	}

	// Define Object for saving data on a file
	Logger log, vtLog, jLog;

	// Add components to new simulation	and create system matrix
	std::cout << std::endl << "Create System matrix" << std::endl;

	Simulation newSim(circElements, om, dt, tf, log);

	std::cout << "Initialization Completed" << std::endl;
	std::cout << "Entering Main Simulation Loop" << std::endl;
	std::cout << std::endl;

	// Get Current Simulation Time
	t = newSim.getTime();

	// Main Simulation Loop
	while (newSim.Step())
	{
		// Save Simulation Step
		vtLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getLeftSideVector()).str();
		jLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getRightSideVector()).str();
		// Get Current Simulation Time
		t = newSim.getTime();

		UpdateProgressBar(t, tf);
	}
	std::cout << "#######################   (100%)" << std::endl;;
	std::cout << std::endl;
	std::cout << "Simulation finished" << std::endl;

	log.WriteLogToFile("log.txt");
	vtLog.WriteLogToFile("data_vt.csv");
	jLog.WriteLogToFile("data_j.csv");

	std::cout << "Saved data points" << std::endl;	
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

void UpdateProgressBar(double t, double tf) {

	if (t / tf <= 0.33) {
		std::cout << "                        (0%)\r";
	}
	else if (t / tf > 0.33 && t / tf <= 0.66) {
		std::cout << "#####                     (33%)\r";
	}
	else if (t / tf > 0.66 && t / tf < 1) {
		std::cout << "#############             (66%)\r";
	}
	else {
		std::cout << "#######################   (100%)";
		std::cout << std::endl;
	}
}