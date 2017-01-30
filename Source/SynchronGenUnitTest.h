#ifndef  SYNCHRONGENUNITTEST_H
#define SYNCHRONGENUNITTEST_H

#include <iostream>
#include <string>
#include "MathLibrary.h"
#include "Simulation.h"
#include "Components.h"
#include "Logger.h"

// EMT generator tests
void SynGenUnitTestBalancedResLoad();
void SynGenUnitTestPhaseToPhaseFault();
void SynGenUnitTestThreePhaseFault();

// Dynamic Phasor generator tests
void SynGenDPUnitTestBalancedResLoad();

#endif
