
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
		// Input
        const  Attribute<MatrixComp>::Ptr mInput;
		//  Output
        const  Attribute<MatrixComp>::Ptr mOutput;

		PMUSignalDevice(String name, Logger::Level logLevel = Logger::Level::off);

		// /// Setter for initial values
        // void setInitialValues(Real input_init, Real state_init, Real output_init);

		/// Operation for adding measurement error
		void MeasurementError(Real time);
		

		Task::List getTasks();

 		class PostStep : public Task {
			public:
				PostStep(PMUSignalDevice& PMU) :
					Task(**PMU.mName + ".PostStep"), mPMU(PMU) {
						mAttributeDependencies.push_back(PMU.mInput);
						mModifiedAttributes.push_back(PMU.mOutput);
					}

				void execute(Real time, Int timeStepCount);

			private:
				PMUSignalDevice& mPMU;
			};
        };
    }
}