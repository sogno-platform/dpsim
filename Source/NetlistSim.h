#ifndef NETLISTSIM_H
#define NETLISTSIM_H

#include <iostream>
#include <string>
#include "MathLibrary.h"
#include "Simulation.h"
#include "Components.h"
#include "Logger.h"
#include "TopologyReader.h"

void readCmdLineArguments(char* &confFilename, int argc, char* argv[]);
void UpdateProgressBar(double t, double tf);
void NetlistSim(int argc, char* argv[]);

#endif
