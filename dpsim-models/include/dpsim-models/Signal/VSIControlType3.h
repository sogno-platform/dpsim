#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_VSIControlDQ.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

    
    class VSIControlType3Parameters :
		public Base::VSIControlParameters,
		public SharedFactory<VSIControlType3Parameters> {
		
		public:
			/// 
            Real Kpv;
            /// 
            Real Kiv;
            /// 
	        Real Kpc;
            /// 
            Real Kic;
			///
            Real VdRef;
            ///
            Real VqRef;
	};

    /// DOC: Controller for a grid-forming power converter, 
	/// formed by one PI (voltage loop) and one PT1 block which
    /// subtitutes the PI controller of the current loop.
	/// The output of the inverter is the equivalent current flowing
    /// into the LC-Filter, e.g. the reference for the current source 
    /// of the inverter
	/// *** This controller does not consider the feedforward terms
	/// Ref.: Yazdani
	class VSIControlType3 :
		public SimSignalComp,
        public Base::VSIControlDQ,
		public SharedFactory<VSIControlType3> {

    private:
        /// Controller Parameters
		std::shared_ptr<VSIControlType3Parameters> mParameters;

        /// Controller variables
        /// state variable of the outer loop (d-component)
        const Attribute<Real>::Ptr mPhi_d;
        /// state variable of the outer loop (q-component)
        const Attribute<Real>::Ptr mPhi_q;
        /// state variable of the inner loop (d-component)
        const Attribute<Real>::Ptr mGamma_d;
        /// state variable of the inner loop (q-component)
        const Attribute<Real>::Ptr mGamma_q;

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

		// input, state and output vectors
		// matrixes of the seperate control loops previous and current
		/// Previous Input
        const Attribute<Matrix>::Ptr mInputPrev;
        /// Current Input
        const Attribute<Matrix>::Ptr mInputCurr;
        /// Previous State
        const Attribute<Matrix>::Ptr mStatePrev;
        /// Current State
        const Attribute<Matrix>::Ptr mStateCurr;
        /// Output
        const Attribute<Matrix>::Ptr mOutput;

    public:
        ///
        explicit VSIControlType3(const String & name, CPS::Logger::Level logLevel) : 
            SimSignalComp(name, name, logLevel),
            mPhi_d(mAttributes->create<Real>("Phi_d", 0)),
			mPhi_q(mAttributes->create<Real>("Phi_q", 0)),
			mGamma_d(mAttributes->create<Real>("Gamma_d", 0)),
			mGamma_q(mAttributes->create<Real>("Gamma_q", 0)),
			mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(6,1))),
    		mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(6,1))),
    		mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(4,1))),
    		mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(4,1))),
    		mOutput(mAttributes->create<Matrix>("output", Matrix::Zero(2,1))) { }

	    /// Sets parameters of the vsi controller
	    void setParameters(std::shared_ptr<Base::VSIControlParameters> parameters) final;

	    /// Initialises the initial state of the vsi controller
	    void initialize(const Complex& Vsref_dq, const Complex& Vcap_dq,
						const Complex& Ifilter_dq, Real time_step, Bool modelAsCurrentSource) final;

	    /// Performs a step to update all state variables and the output
	    Complex step(const Complex& Vcap_dq, const Complex& Ifilter_dq) final;
    };

}
}