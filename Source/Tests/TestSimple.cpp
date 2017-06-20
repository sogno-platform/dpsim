#include "../Simulation.h"

using namespace DPsim;

int main(int argc, char *argv[]) {
	Logger logNone(LogLevel::NONE), llog;

	std::vector<BaseComponent*> comps;
	comps.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
	comps.push_back(new LinearResistor("r_line", 1, 2, 1));
	comps.push_back(new Inductor("l_line", 2, 3, 1));	
	comps.push_back(new LinearResistor("r_load", 3, 0, 1000));

	Real timeStemp = 0.001;
	Simulation sim(comps, 2*M_PI*50, timeStemp, 0.3, logNone);

	while (sim.step(logNone, llog, logNone)) {
		sim.increaseByTimeStep();
	}
	llog.WriteLogToFile("TestSimple.csv");
}
