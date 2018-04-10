#include <iostream>
#include <vector>
#include "Definitions.h"
#include "Component.h"
//#include "Logger.h"
#include "SystemModel.h"
//#include "ExternalInterface.h"
#include <ida/ida.h>
#include "nvector/nvector_serial.h"

#define NVECTOR_DATA(vec) NV_DATA_S (vec) // returns pointer to the first element of array vec

using namespace DPsim ;

	/// Ground node
	const Int GND = -1;


	class DAESimulation {

	protected:
		/// Simulation name
		String mName;
		SystemModel mSystemModel;
		/// Stores a list of circuit elements that are used to generate the system matrix
		Component::List mComponents;
		/// Circuit list vector
		std::vector<Component::List> mComponentsVector;
		// stores the required offsets for adding new components to the residual vector
		std::vector<int> offsets; 
		///TO-DO: Implement the offset based on previous components
		///list of Nodes for residual calculation
		Node::List mNodes;

	public:
		/// Create DAE System
		DAESimulation(String name, Component::List comps, Real om, Real dt, SimulationType simType = SimulationType::DynPhasor);
		virtual ~DAESimulation() { };

		void initialize(Component::List comps);

		void addSystemTopology(Component::List newComps);

		void switchSystemMatrix(Int systemMatrixIndex);

		// Residual Function to be used by IDA
		int DAE_residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data);

		/// Run simulation until total time is elapsed.
		void run();
	};


