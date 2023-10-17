#include <dpsim-models/Signal/HydroTurbineGovernor.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::HydroTurbineGovernor::HydroTurbineGovernor(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel){ }

void HydroTurbineGovernor::setParameters(std::shared_ptr<Base::GovernorParameters> parameters){
    
    if (auto params = std::dynamic_pointer_cast<Signal::HydroGorvernorParameters>(parameters)){
		mParameters = params;

        SPDLOG_LOGGER_INFO(mSLog, 
            "Hydro Governor parameters: "
			"\nOmRef: {:e}"
			"\nR: {:e}"
            "\nT1: {:e}"
			"\nT2: {:e}"
			"\nT3: {:e}"
			"\nPmax: {:e}"
			"\nPmin: {:e}",
			mParameters->OmRef, mParameters->R, mParameters->T1, 
            mParameters->T2, mParameters->T3, 
            mParameters->Pmax, mParameters->Pmin);
    } else {
	    std::cout << "Type of parameters class of " << this->name() << " has to be HydroGorvernorParameters!" << std::endl;
	    throw CPS::TypeException();
    }
}

void HydroTurbineGovernor::initialize(Real Pref){

	if (Pref>=mParameters->Pmin && Pref<=mParameters->Pmax) {
        mPgv=Pref;
        mPref=Pref;
        mDelOm = 0;
        mX1 = 0;
        mX1_next = 0;
        mX2 = 0;
        mX2_next = 0;
        SPDLOG_LOGGER_INFO(mSLog, 
            "Hydro Governor initial values: \n"
			"\nPref: {:f}"
			"\nDelOm: {:f}"
			"\nX1: {:f}"
			"\nX2: {:f}"
			"\nPgv: {:f}",
			Pref, mDelOm, mX1, mX2, mPgv);
    } else {
		SPDLOG_LOGGER_INFO(mSLog, 
			"\nP_ref of hydro governor {} should be a value between 0 and 1 in pu",
			this->name());
		throw CPS::TypeException();
	}
}

Real HydroTurbineGovernor::step(Real Omega, Real dt){
    // FIXME: Ein problem entsteht bei der Zerlegung wenn T1=t3 (gleiche polstellen, es muss eine andere Zerlegung benutzt werden)
    // TODO: move cA and cB to initialize() to avoid recomputation in each simulation step
    const Real cA = (mParameters->T1 - mParameters->T2) / (mParameters->T1 - mParameters->T3);
    const Real cB = (mParameters->T2 - mParameters->T3) / (mParameters->T1 - mParameters->T3);
    
    //Calculation of frequency deviation
    mDelOm=mParameters->OmRef-Omega;
    
    // Set the values of state variables calculated in last step as atual values for k
    mX1=mX1_next;
    mX2=mX2_next;

    // Calculation State variables for k+1 with integrators
    mX1_next= mX1 + dt / mParameters->T1 * (mDelOm - mX1);
    mX2_next= mX2 + dt / mParameters->T3 * (mDelOm - mX2);
    
    // Output of the governor before limiter, values from k are used to caltulate output Pgv(k)
    mPgv = mPref + 1. / mParameters->R * ( mX1 * cA + mX2 * cB);

    if (mPgv>mParameters->Pmax)
        mPgv = mParameters->Pmax;
    if(mPgv<mParameters->Pmin)
        mPgv = mParameters->Pmin;

	return mPgv;
}

    