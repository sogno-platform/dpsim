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

/* std::size_t is the largest data type. No container can store
 * more than std::size_t elements. Define the number of switches
 * as the log_2 of this value so that we end up with maximally
 * std::size_t matrices. The overhead of statically defining this
 * value should be minimal.
 **/
#define SWITCH_NUM sizeof(std::size_t)*8

namespace DPsim {
	/// Solver class using Modified Nodal Analysis (MNA).
	class SolverParametersMNA : public SolverParameters {
	protected: 
		// #### General simulation settings ####
		/// Simulation domain, which can be dynamic phasor (DP) or EMT
		CPS::Domain mDomain;
		
        Solver::Behaviour mSolverBehaviour = Solver::Behaviour::Simulation;

		/// Determines if the network should be split
		/// into subnetworks at decoupling lines.
		/// If the system is split, each subsystem is
		/// solved by a dedicated MNA solver.
		const CPS::Attribute<Bool>::Ptr mSplitSubnets;

        /// Determines if the system matrix is split into
		/// several smaller matrices, one for each frequency.
		/// This can only be done if the network is composed
		/// of linear components that do no create cross
		/// frequency coupling.
		Bool mFreqParallel = false;

        /// Enable recomputation of system matrix during simulation
		Bool mSystemMatrixRecomputation = false;

		/// Determines if steady-state initialization
		/// should be executed prior to the simulation.
		/// By default the initialization is disabled.
		const CPS::Attribute<Bool>::Ptr mSteadyStateInit;

        	// #### Initialization ####
		/// steady state initialization time limit
		Real mSteadStIniTimeLimit = 10;
		/// steady state initialization accuracy limit
		Real mSteadStIniAccLimit = 0.0001;

        DirectLinearSolverImpl mDirectImpl = DirectLinearSolverImpl::Undef;




	public:

		SolverParametersMNA() {}
		
		/// Destructor
		virtual ~SolverParametersMNA() {};

        // #### Simulation Settings ####
		///
		void setDomain(CPS::Domain domain = CPS::Domain::DP) { mDomain = domain; }

        void setSolverAndComponentBehaviour(Solver::Behaviour behaviour) { mSolverBehaviour = behaviour; } 
		///
        void doSplitSubnets(Bool splitSubnets = true) { **mSplitSubnets = splitSubnets; }
		///
        /// Compute phasors of different frequencies in parallel
		void doFrequencyParallelization(Bool value) { mFreqParallel = value; }
		///
        void doSystemMatrixRecomputation(Bool value) { mSystemMatrixRecomputation = value; }
        ///
        void setDirectLinearSolverImplementation(DirectLinearSolverImpl directImpl) { mDirectImpl = directImpl; }     


        // #### Initialization ####
		/// activate steady state initialization
		void doSteadyStateInit(Bool f) { **mSteadyStateInit = f; }
		/// set steady state initialization time limit
		void setSteadStIniTimeLimit(Real v) { mSteadStIniTimeLimit = v; }
		/// set steady state initialization accuracy limit
		void setSteadStIniAccLimit(Real v) { mSteadStIniAccLimit = v; }

        // #### Getter ####
        CPS::Bool getSplitSubnets() const { return **mSplitSubnets; }
        CPS::Bool getFreqParallel() const { return mFreqParallel; }
        CPS::Bool getSystemMatrixRecomputation() const { return mSystemMatrixRecomputation; }
        CPS::Bool getSteadyStateInit() const { return **mSteadyStateInit; }
        CPS::Real getSteadyStateInitTimeLimit() const { return mSteadStIniTimeLimit; }
        CPS::Real getSteadyStateInitAccLimit() const { return mSteadStIniAccLimit; }
    };
}
