#include <dpsim-models/Signal/PMUSignalDevice.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

using namespace CPS;
using namespace CPS::Signal;

PMUSignalDevice::PMUSignalDevice(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    /// CHECK: Which of these really need to be attributes?
    mInput(mAttributes->createDynamic<MatrixComp>("input")), 

    //*** Important: the output attribute is a matrix component, which should be initialized as Zero(1,1)
    mOutput(mAttributes->create<MatrixComp>("output", MatrixComp::Zero(1,1))){ }

   
// }


void PMUSignalDevice::MeasurementError(Real time){

    // GSL random number generator
    const gsl_rng_type *T;
    const gsl_rng *r;
    double val;

    // Setup environment
    T = gsl_rng_taus;
    r = gsl_rng_alloc (T);
    val = gsl_ran_gaussian (r, 2.0);

    SPDLOG_LOGGER_INFO(mSLog, "GSL value: gsl= {}", val);

    SPDLOG_LOGGER_INFO(mSLog,"Input values: input = {}", **mInput);
    

    /// adding the measurement error
    // noise is considered as a 10% error
     **mOutput =**mInput*1.1;
    // **mOutput=gsl_ran_gaussian(r,sigma)+**mInput;

    SPDLOG_LOGGER_INFO(mSLog,"Output values: output = {}:", **mOutput);
}

void PMUSignalDevice::PostStep::execute(Real time, Int timeStepCount) {
 	mPMU.MeasurementError(time);
}

Task::List PMUSignalDevice::getTasks() {
	return Task::List({std::make_shared<PMUSignalDevice::PostStep>(*this)});
}