#include <iostream>
#include <vector>
#include "Definitions.h"
#include "Component.h"
//#include "Logger.h"
#include "SystemModel.h"
//#include "ExternalInterface.h"
#include <ida/ida.h>

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

	public:
		/// Creates DAE System
		DAESimulation(String name, Component::List comps, Real om, Real dt, SimulationType simType = SimulationType::DynPhasor);
		virtual ~DAESimulation() { };

		void initialize(Component::List comps);

		void addSystemTopology(Component::List newComps);

		void switchSystemMatrix(Int systemMatrixIndex);

		/// Run simulation until total time is elapsed.
		void run();
	};


