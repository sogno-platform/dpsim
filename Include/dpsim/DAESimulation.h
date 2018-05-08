
#pragma once 
#include <iostream>
#include <vector>
#include <list>
#include "Simulation.h"
#include "cps/SystemTopology.h"
//#include "Logger.h"
#include <ida/ida.h>
#include <ida/ida_dense.h>
#include <nvector/nvector_serial.h>

#define NVECTOR_DATA(vec) NV_DATA_S (vec) // returns pointer to the first element of array vec

using namespace DPsim ;

	class DAESimulation : public Simulation{
 //inherit from Simulation?
	protected:
		/// Simulation name
		String mName;
		///stores Simulation parameters
		SystemTopology DAESys;
		// stores the required offsets for adding new components to the residual vector
		std::vector<int> offsets; 
		///TO-DO: Implement the offset based on previous components

	public:
		/// #### Create DAE System ####
		//TO-DO: initilaize state vector with nodes and components;
		DAESimulation(String name,  SystemTopology system, Real dt, Real tfinal);
		virtual ~DAESimulation() { };
		//TO-DO: initialize Components/Nodes with inital values
		void initialize(Component::List comps);
		// Residual Function to be used by IDA
		int DAE_residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data);
		/// Run simulation until total time is elapsed.
		void run();
	};


