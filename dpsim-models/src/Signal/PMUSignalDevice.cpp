#include <dpsim-models/Signal/PMUSignalDevice.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <time.h>
using namespace CPS;
using namespace CPS::Signal;

PMUSignalDevice::PMUSignalDevice(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    /// CHECK: Which of these really need to be attributes?
    mInput(mAttributes->createDynamic<MatrixComp>("input")), 

    //*** Important: the output attribute is a matrix component, which should be initialized as Zero(1,1)
    mOutput(mAttributes->create<MatrixComp>("output", MatrixComp::Zero(1,1))){ }

   
// }

void PMUSignalDevice::setParameters(Real Sigma){
    mSigma=Sigma;
}

void PMUSignalDevice::MeasurementError(Real time){

    // Relative standard measurement uncertainty
    double relMeasUncertainty;
    
    // GSL random number generator
    gsl_rng *r=gsl_rng_alloc(gsl_rng_default);

    // To avoid the seed of random number always selected as the same, we use the UTC time to generate the seed.
    // Time consumption of the calculations betweens elements is small, here the "tv_nsec" can distinct the time difference within nano second.
    struct timespec ts;
    timespec_get(&ts,TIME_UTC);
    gsl_rng_set(r,ts.tv_nsec);

    // The relative measurement uncertainty is gaussian distributed, mean value 0, standard deviation mSigma.
    relMeasUncertainty = gsl_ran_gaussian (r, mSigma);

    // Output = Input + error 
    //        = Input + k*Sigma 
    //        = Input + Input*relMeasUncertainty
     **mOutput =**mInput*(1+relMeasUncertainty);

    // To log the GSL random number, mInput and mOutput.
    SPDLOG_LOGGER_INFO(mSLog, "GSL value: gsl= {}", relMeasUncertainty);
    SPDLOG_LOGGER_INFO(mSLog,"Input values: input = {}", **mInput);
    SPDLOG_LOGGER_INFO(mSLog,"Output values: output = {}:", **mOutput);
}

void PMUSignalDevice::PostStep::execute(Real time, Int timeStepCount) {
 	mPMU.MeasurementError(time);
}

Task::List PMUSignalDevice::getTasks() {
	return Task::List({std::make_shared<PMUSignalDevice::PostStep>(*this)});
}