#include <dpsim-models/Signal/PMUSignalDevice.h>

using namespace CPS;
using namespace CPS::Signal;

PMUSignalDevice::PMUSignalDevice(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    /// CHECK: Which of these really need to be attributes?
    mInput(Attribute<MatrixComp>::createDynamic("input", mAttributes)),

    //*** Important: the output attribute is a matrix component, which should be initialized as Zero(1,1)
    mOutput(Attribute<MatrixComp>::create("output", mAttributes, MatrixComp::Zero(1,1))){ }

   
// }


void PMUSignalDevice::MeasurementError(Real time){
    

    mSLog->info("Input values: input = {}", **mInput);
    

    /// adding the measurement error
    // noise is considered as a 10% error
    **mOutput =**mInput*1.1;

    mSLog->info("Output values: output = {}:", **mOutput);
}

void PMUSignalDevice::PostStep::execute(Real time, Int timeStepCount) {
 	mPMU.MeasurementError(time);
}

Task::List PMUSignalDevice::getTasks() {
	return Task::List({std::make_shared<PMUSignalDevice::PostStep>(*this)});
}