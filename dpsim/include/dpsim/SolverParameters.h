
/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once
#include <dpsim/Definitions.h>
#include <dpsim-models/Attribute.h>

namespace DPsim {
	
	/// Base class for more specific solvers such as MNA, ODE or IDA.
	class SolverParameters{
	protected:
		/// Time step for fixed step solvers
        const CPS::Attribute<Real>::Ptr mTimeStep;

		// #### Initialization ####
		
		/// If this is false, all voltages are initialized with zero
		Bool mInitFromNodesAndTerminals = true;

	public:

		SolverParameters() {}

		virtual ~SolverParameters() { }

		void setTimeStep(Real timeStep) { **mTimeStep = timeStep; }
		
		// #### Initialization ####
		/// activate powerflow initialization
		void setInitFromNodesAndTerminals(Bool f) { mInitFromNodesAndTerminals = f; }
		

		// #### Getter ####
        Bool getTimeStep() {return **mTimeStep;}

		Bool getInitFromNodesAndTerminals() {return mInitFromNodesAndTerminals;}
 		
	};
}
