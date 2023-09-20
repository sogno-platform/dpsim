#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

	class SteamTurbineGovernor:
		public SimSignalComp,
		public SharedFactory<SteamTurbineGovernor> {

    private:
    // ### Steam turbine governor parameters, where Omega_ref,  P_ref are constant and T_1=0 ###
    //Controller is K(1+sT_2)
    //Droop, the value 1/K in the controller K(1+sT_2)
    Real mR;
    //T_2 related to the differentiator in the controlle K(1+sT_2)
    Real mT2;
    //Time constant T_3 of the actuator in the Governor
    Real mT3;

    // ### Physical constraints ###
    //Maximum growth rate
    Real mdPmax;
    //Minimum decay rate
    Real mdPmin;
    //Maximum mechanical power(pu)
    Real mPmax;
    //Minimum mechanical power (pu)
    Real mPmin;

    // ### Setpoints of the machine ###
    //Setpoint for omega (pu). It is adviced to choose Om_ref=1
    Real mOmRef;
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
	void setParameters(Real OmRef, Real R, Real T2, Real T3, 
                        Real dPmax, Real dPmin, Real Pmax, Real Pmin);

	/// Initialises the initial state of the turbine
	void initialize(Real Pref) override;

	/// Performs a step to update all state variables and the output
	Real step(Real Omega, Real dt);
        };

}
}