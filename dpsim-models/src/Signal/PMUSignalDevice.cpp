#include <dpsim-models/Signal/PMUSignalDeivice.h>

using namespace CPS;
using namespace CPS::Signal;

PMUSignalDevice::PMUSignalDevice::(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    mInputRef(Attribute<Real>::createDynamic("input_ref", mAttributes)),
    /// CHECK: Which of these really need to be attributes?
    mInputPrev(Attribute<Real>::create("input_prev", mAttributes)),
    mStatePrev(Attribute<Real>::create("state_prev", mAttributes)),
    mOutputPrev(Attribute<Real>::create("output_prev", mAttributes)),
    mInputCurr(Attribute<Real>::create("input_curr", mAttributes)),
    mStateCurr(Attribute<Real>::create("state_curr", mAttributes)),
    mOutputCurr(Attribute<Real>::create("output_curr", mAttributes)){ }

void PMUSignalDevice::setInitialValues(Real input_init, Real State_init, Real output_init){
    **mInputCurr = input_init;
    **mStateCurr = state_init;
    **mOutputCurr = output_init;
    mSLog->info("Initial values:");
    mSLog->info("inputCurrInit = {}, stateCurrInit = {}, outputCurrInit = {}", **mInputCurr, **mStateCurr, **mOutputCurr);
   
}
void PMUSignalDevice::MeasurementError(){
    **mInputCurr = **mInputRef;

    mSLog->info("Input values: inputCurr = {}, inputPrev = {}, statePrev = {}", **mInputCurr, **mInputPrev, **mStatePrev);
    
    /// adding the measurement error
    **mStateCurr =**mStatePrev * 1.1
    **mOutputCurr = **mStateCurr;

    mSLog->info("State values: stateCurr = {}", **mStateCurr);
    mSLog->info("Output values: outputCurr = {}:", **mOutputCurr);
}