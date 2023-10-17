#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_Turbine.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

    class SteamTurbineParameters :
		public Base::TurbineParameters,
		public SharedFactory<SteamTurbineParameters> {
		
		public:
			/// Power fraction of a high pressure stage
            Real Fhp;
            /// Power fraction of an intermediate pressure stage
            Real Fip;
            /// Power fraction of a low pressure stage
            Real Flp;
	        /// Time constant of main inlet volume and steam chest (s)
	        Real Tch;
	        /// Time constant of reheater (s)
	        Real Trh;
	        /// Time constant of cross over piping and LP inlet volumes(s)
	        Real Tco;
	};

    /// Steam turbine model
    /// Single reheat turbine systems with three turbine stages coupled on one shaft (tandem compound)
	/// Ref.: MATPAT Paper
	class SteamTurbine :
		public SimSignalComp,
        public Base::Turbine,
		public SharedFactory<SteamTurbine> {

    private:
        /// Turbine Parameters
		std::shared_ptr<SteamTurbineParameters> mParameters;

        /// Turbine variables
        /// Power of low pressure stage at time step k+1
        Real mPlp_next;
        /// Power of intermediate pressure stage at time step k+1;
        Real mPip_next;
        /// Power of high pressure stage at time step k+1;
        Real mPhp_next;

        /// Power of low pressure stage at time step k
        Real mPlp;
        /// Power of intermediate pressure stage at time step k;
        Real mPip;
        /// Power of high pressure stage at time step k;
        Real mPhp;

        /// Mechanical output power at time k (in pu is queal to torque)
        Real mPm=0;

    public:
        ///
        explicit SteamTurbine(const String & name) : SimSignalComp(name, name) { }

	    /// Constructor with log level
	    SteamTurbine(const String & name, CPS::Logger::Level logLevel);

	    /// Sets Parameters of the turbine
	    void setParameters(std::shared_ptr<Base::TurbineParameters> parameters) final;

	    /// Initialises the initial state of the turbine
	    void initialize(Real Pminit) final;

	    /// Performs a step to update all state variables and the output
	    Real step(Real Pgv, Real dt) final;
    };

}
}