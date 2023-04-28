// Attribute und Funktionen in einem anderen Branch

#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <bitset>

#include <DPsim.h>

#include <dpsim/Config.h>
#include <dpsim/Solver.h>
#include <dpsim/SolverParameters.h>
#include <dpsim/DataLogger.h>
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/SimPowerComp.h>

using namespace CPS;
using namespace DPsim;

/* std::size_t is the largest data type. No container can store
 * more than std::size_t elements. Define the number of switches
 * as the log_2 of this value so that we end up with maximally
 * std::size_t matrices. The overhead of statically defining this
 * value should be minimal.
 **/
#define SWITCH_NUM sizeof(std::size_t)*8

namespace DPsim {
	/// Solver class using Modified Nodal Analysis (MNA).
	class SolverParametersNRP : public SolverParameters {
	protected: 
		// #### General simulation settings ####
		/// Simulation domain, which can be dynamic phasor (DP) or EMT
	


	public:
		
		/// Destructor
		virtual ~SolverParametersNRP() {};

    
    };
}
