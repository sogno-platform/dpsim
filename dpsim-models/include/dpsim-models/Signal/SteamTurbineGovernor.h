/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_Governor.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

    class SteamGorvernorParameters :
		public Base::GovernorParameters,
		public SharedFactory<SteamGorvernorParameters> {

        public:
            // Droop, the value 1/K in the controller K(1+sT_2)
            Real R;
            // T_2 related to the differentiator in the controlle K(1+sT_2)
            Real T2;
            // Time constant T_3 of the actuator in the Governor
            Real T3;
        
            // ### Physical constraints ###
            //Maximum growth rate
            Real dPmax;
            //Minimum decay rate
            Real dPmin;
            //Maximum mechanical power(pu)
            Real Pmax;
            //Minimum mechanical power (pu)
            Real Pmin;

            // Setpoint for omega (pu). It is adviced to choose Om_ref=1
            Real OmRef;
    };

    /// Steam turbine governor, where Omega_ref, P_ref are constant and T_1=0
    /// Ref.: MATPAT Paper
    class SteamTurbineGovernor:
	    public SimSignalComp,
        public Base::Governor,
	    public SharedFactory<SteamTurbineGovernor> {

        private:
            /// Governor Parameters
		    std::shared_ptr<SteamGorvernorParameters> mParameters;
    
            // ### Setpoints of the machine ###
            //Setpoint for mechanical Power (pu)
            Real mPref;
    
            // ### Variables at time step k-1 ###
            Real mDelOm_prev;    
    
            // ### Variables at time step k ###
            // Delta Omega = Omega_ref-Omega_meas at k
            Real mDelOm;
            //Variable after the rate limiter and before integrator at k
            Real mDelPgv;
            // The outpur of the Governor at k
            Real mPgv;
    
            // ### Variables at time step k+1 ###
            // The outpur of the PT1 with limiters at k+1 (Governor output)
            Real mPgv_next;

        public:
            ///
            explicit SteamTurbineGovernor(const String & name) : SimSignalComp(name, name) { }

	        /// Constructor with log level
	        SteamTurbineGovernor(const String & name, CPS::Logger::Level logLevel);

	        /// Sets Parameters of the turbine
	        void setParameters(std::shared_ptr<Base::GovernorParameters> parameters) final;

	        /// Initialises the initial state of the turbine
	        void initialize(Real Pref) final;

	        /// Performs a step to update all state variables and the output
	        Real step(Real Omega, Real dt) final;
    };

}
}