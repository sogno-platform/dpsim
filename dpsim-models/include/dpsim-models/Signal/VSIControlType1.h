#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_VSIControlDQ.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

    class VSIControlType1Parameters :
		public Base::VSIControlParameters,
		public SharedFactory<VSIControlType1Parameters> {
		
		public:
			/// 
            Real Kpv;
            /// 
            Real Kiv;
            /// 
            Real Kic;
	        /// 
	        Real Kpi;
            ///
            Real Vdref;
            ///
            Real Vqref;
	};

    /// Doc: TODO
	/// Ref.: TODO
	class VSIControlType1 :
		public SimSignalComp,
        public Base::VSIControlDQ,
		public SharedFactory<VSIControlType1> {

    private:
        /// Turbine Parameters
		std::shared_ptr<VSIControlType1Parameters> mParameters;

        /// Controller variables
        /// state variable of the outer loop (d-component)
        Real mPhi_d;
        /// state variable of the outer loop (q-component)
        Real mPhi_q;
        /// state variable of the inner loop (d-component)
        Real mGamma_d;
        /// state variable of the inner loop (q-component)
        Real mGamma_q;

        // State space matrices
		/// matrix A of state space model
		Matrix mA = Matrix::Zero(4, 4);
		/// matrix B of state space model
		Matrix mB = Matrix::Zero(4, 6);
		/// matrix C of state space model
		Matrix mC = Matrix::Zero(2, 4);
		/// matrix D of state space model
		Matrix mD = Matrix::Zero(2, 6);

        /// Trapedzoidal based state space matrix A
		Matrix mATrapezoidal;
		/// Trapedzoidal based state space matrix B
		Matrix mBTrapezoidal;
		/// Trapedzoidal based state space matrix Matrix::Zero(2,1)
		Matrix mCTrapezoidal;

    public:
        ///
        explicit VSIControlType1(const String & name) : SimSignalComp(name, name) { }

	    /// Constructor with log level
	    VSIControlType1(const String & name, CPS::Logger::Level logLevel);

	    /// Sets Parameters of the turbine
	    void setParameters(std::shared_ptr<Base::VSIControlParameters> parameters) final;

	    /// Initialises the initial state of the turbine
	    void initialize() final;

	    /// Performs a step to update all state variables and the output
	    Real step() final;
    };

}
}