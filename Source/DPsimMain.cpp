#include <iostream>
#include <string>
#include "CIMReader.h"
#include "Simulation.h"

using namespace DPsim;

int main(int argc, char* argv[]) {
	// TODO: parse frequency, timestep etc. as command-line flags
	CIMReader reader(2*PI*50);
	Logger log("cim.log"), llog("lvector-cim.csv"), rlog("rvector-cim.csv");
	for (int i = 1; i < argc; i++) {
		if (!reader.addFile(argv[i]))
			std::cerr << "Failed to read file " << argv[i] << std::endl;
	}
	std::vector<BaseComponent*> components = reader.mapComponents();
	Simulation sim(components, 2*PI*50, 0.001, 0.3, log);
	std::cout << "Start simulation." << std::endl;
	while (sim.step(log, llog, rlog))
		sim.increaseByTimeStep();
	std::cout << "Simulation finished." << std::endl;
	return 0;	
}

