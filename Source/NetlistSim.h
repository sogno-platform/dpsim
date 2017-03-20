#ifndef NETLISTSIM_H
#define NETLISTSIM_H

#include <iostream>
#include <string>
#include "MathLibrary.h"
#include "Simulation.h"
#include "Components.h"
#include "Logger.h"
#include "TopologyReader.h"

namespace DPsim {

	void readCmdLineArguments(char* &confFilename, int argc, char* argv[]);
	void NetlistSim(int argc, char* argv[]);
}

#endif
