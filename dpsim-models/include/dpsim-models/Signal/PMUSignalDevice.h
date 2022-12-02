
#pragma once

#include <vector>

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>


namespace CPS {
namespace Signal {

    class PMUSignalDevice:
        public SimSignalComp,
		public SharedFactory<PMUSignalDevice> 
        {
	protected:
	public:

		///FIXME: This is never explicitely set to reference anything, so the outside code is responsible for setting up the reference.
		const Attribute<Real>::Ptr mInputRef;

		/// Previous Input
        const Attribute<Real>::Ptr mInputPrev;
        /// Current Input
        const Attribute<Real>::Ptr mInputCurr;
        /// Previous State
        const Attribute<Real>::Ptr mStatePrev;
        /// Current State
        const Attribute<Real>::Ptr mStateCurr;
        /// Previous Output
        const Attribute<Real>::Ptr mOutputPrev;
        /// Current Output
        const Attribute<Real>::Ptr mOutputCurr;

		PMUSignalDevice(String name, Logger::Level logLevel = Logger::Level::off);

		/// Setter for initial values
        void setInitialValues(Real input_init, Real state_init, Real output_init);

		/// Operation for adding measurement error
		void MeasurementError()
        };
    }
    }