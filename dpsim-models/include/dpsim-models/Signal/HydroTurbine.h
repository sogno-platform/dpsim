#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

	class HydroTurbine :
		public SimSignalComp,
		public SharedFactory<HydroTurbine> {

    private:
    
    /// Water starting time (s) of the model (1- s T_w)/ (1+ 0,5 s T_w ) = -2 + 3 / (1+0,5 T_w s)
  	Real mTw;

    ///Turbine variables
    ///State variable after the PT1 with time constant 0.5T_w at time step k
    Real mX1;
    ///State variable after the PT1 with time constant 0.5T_w at time at time step k+1
    Real mX1_next;

    ///Output mechanical power of the tubine in pu =mechanical torque in pu at k 
    Real mPm;

    public:
    ///
  explicit HydroTurbine(const String & name) : SimSignalComp(name, name) { }

	/// Constructor with log level
	HydroTurbine(const String & name, CPS::Logger::Level logLevel);

	/// Sets Parameters of the turbine
	void setParameters(Real Tw);

	/// Initialises the initial state of the turbine
	void initialize(Real Pminit) override;

	///Gets the current input Pgv(k) from fovernor and returns the current output of the system (during k)
  /// and if needed caltulates the state variables for the next step
	Real step(Real Pgv, Real dt);
        };

}
}