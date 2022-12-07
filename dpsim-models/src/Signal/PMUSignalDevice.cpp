#include <dpsim-models/Signal/PMUSignalDevice.h>

using namespace CPS;
using namespace CPS::Signal;

PMUSignalDevice::PMUSignalDevice(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    /// CHECK: Which of these really need to be attributes?
    mInput(Attribute<MatrixComp>::createDynamic("input", mAttributes)),
    mOutput(Attribute<MatrixComp>::create("output", mAttributes)){ }

// void PMUSignalDevice::setInitialValues(Real input_init, Real State_init, Real output_init){
//     **mInputCurr = input_init;
//     **mStateCurr = state_init;
//     **mOutputCurr = output_init;
//     mSLog->info("Initial values:");
//     mSLog->info("inputCurrInit = {}, stateCurrInit = {}, outputCurrInit = {}", **mInputCurr, **mStateCurr, **mOutputCurr);
   
// }


void PMUSignalDevice::MeasurementError(Real time){
    //**mInput = **mInputRef;

    mSLog->info("Input values: input = {}", **mInput);
    

    /// adding the measurement error
    // auto noise=Attribute<Matrixcomp>::create("noise", mAttributes)
    // **noise=**mInput
    **mOutput =**mInput;

    mSLog->info("Output values: output = {}:", **mOutput);
}

void PMUSignalDevice::PostStep::execute(Real time, Int timeStepCount) {
 	mPMU.MeasurementError(time);
}

Task::List PMUSignalDevice::getTasks() {
	return Task::List({std::make_shared<PMUSignalDevice::PostStep>(*this)});
}