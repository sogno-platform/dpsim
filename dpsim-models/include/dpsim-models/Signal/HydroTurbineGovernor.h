#pragma once
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

class HydroTurbineGovernor:
        public SimSignalComp,
        public SharedFactory<HydroTurbineGovernor> {

    private:
    // ### Hydro turbine governor parameters, where Omega_ref,  P_ref are constant and T_1=0 ###
    //Governor is K (1+sT2)/ ( (1+sT1)(1+sT3) ) = 1/R ( A / (1+sT1) + B / (1+sT3))
    //K = 1/R where R is the droop
    Real mR;
    //T1
    Real mT1;
    //T2
    Real mT2;
    //T3
    Real mT3;

    //Maximum mechanical power(pu)
    Real mPmax;
    //Minimum mechanical power (pu)
    Real mPmin;

    //Setpoint for omega (pu). It is adviced to choose Om_ref=1
    Real mOmRef;
    //Setpoint for mechanical Power (pu)
    Real mPref;

    // Delta Omega = Omega_ref-Omega_meas at k
    Real mDelOm;

    // State Variable of T1 PT1 at k
    Real mX1;
    // State Variable of T1 PT1 at k+1
    Real mX1_next;

    // State Variable of T3 PT1 at k
    Real mX2;
    // State Variable of T3 PT1 at k+1
    Real mX2_next;

    // The outpur of the Governor at k
    Real mPgv;

    public:
    ///
    explicit HydroTurbineGovernor(const String & name) : SimSignalComp(name, name) { }

	/// Constructor with log level
	HydroTurbineGovernor(const String & name, CPS::Logger::Level logLevel);

	/// Sets Parameters of the turbine
	void setParameters(Real OmRef, Real R, Real T1, Real T2, Real T3,
                                         Real Pmax, Real Pmin);

	/// Initialises the initial state of the turbine
	void initialize(Real Pref) override;

	/// Performs a step to update all state variables and the output
	Real step(Real Omega, Real dt);
        };

}
}