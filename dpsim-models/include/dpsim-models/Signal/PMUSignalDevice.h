
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
		
		//***PMUSignalDevice has two attributes:Input and Output***//
		const  Attribute<MatrixComp>::Ptr mInput;
        const  Attribute<MatrixComp>::Ptr mOutput;
		Real mSigma;

		//Constructor 
		PMUSignalDevice(String name, Logger::Level logLevel = Logger::Level::off);

		//Operation for adding measurement error
		void MeasurementError(Real time);

	    void setParameters(Real Sigma);
		
		//***Get task list for the solver***//
		Task::List getTasks();

		//***The process of the solve is recognized as a PostStep.***//
 		class PostStep : public Task {
			public:
				PostStep(PMUSignalDevice& PMU) :
					Task(**PMU.mName + ".Poststep"), mPMU(PMU) {
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