#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_VSIControlDQ.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

    class VSIControlType2Parameters :
		public Base::VSIControlParameters,
		public SharedFactory<VSIControlType2Parameters> {
		
		public:
			/// 
            Real Kpv;
            /// 
            Real Kiv;
            /// 
            Real Kic;
	        /// 
	        Real Kpi;
	};

    /// Doc: TODO
	/// Ref.: TODO
	class VSIControlType2 :
		public SimSignalComp,
        public Base::VSIControlDQ,
		public SharedFactory<VSIControlType2> {

    private:
        /// Turbine Parameters
		std::shared_ptr<VSIControlType2Parameters> mParameters;

        /// Controller variables
        /// state variable of the outer loop (d-component)
        Real mPhi_d;
        /// state variable of the outer loop (q-component)
        Real mPhi_q;
        /// state variable of the inner loop (d-component)
        Real mGamma_d;
        /// state variable of the inner loop (q-component)
        Real mGamma_q;

    public:
        ///
        explicit VSIControlType2(const String & name) : SimSignalComp(name, name) { }

	    /// Constructor with log level
	    VSIControlType2(const String & name, CPS::Logger::Level logLevel);

	    /// Sets Parameters of the turbine
	    void setParameters(std::shared_ptr<Base::VSIControlParameters> parameters) final;

	    /// Initialises the initial state of the turbine
	    void initialize() final;

	    /// Performs a step to update all state variables and the output
	    Real step() final;
    };

}
}