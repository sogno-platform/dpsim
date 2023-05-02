#pragma once

#include <dpsim/Definitions.h>
#include <dpsim-models/Attribute.h>
#include <dpsim/Solver.h>

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

        CPS::DirectLinearSolverImpl mDirectImpl = CPS::DirectLinearSolverImpl::Undef;




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
        void setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl directImpl) { mDirectImpl = directImpl; }     


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
