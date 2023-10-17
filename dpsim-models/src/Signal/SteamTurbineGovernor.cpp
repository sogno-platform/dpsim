#include <dpsim-models/Signal/SteamTurbineGovernor.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbineGovernor::SteamTurbineGovernor(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { }

void SteamTurbineGovernor::setParameters(std::shared_ptr<Base::GovernorParameters> parameters) {
    
	if (auto params = std::dynamic_pointer_cast<Signal::SteamGorvernorParameters>(parameters)){
		mParameters = params;
		SPDLOG_LOGGER_INFO(mSLog, 
			"Steam Governor parameters: "
				"\nOmRef: {:e}"
				"\nR: {:e}"
				"\nT2: {:e}"
				"\nT3: {:e}"
				"\ndPmax: {:e}"
				"\ndPmin: {:e}"
				"\nPmax: {:e}"
				"\nPmin: {:e}",
				mParameters->OmRef, mParameters->R, mParameters->T2,
				mParameters->T3, mParameters->dPmax, mParameters->dPmin,
				mParameters->Pmax, mParameters->Pmin);
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be SteamGorvernorParameters!" << std::endl;
		throw CPS::TypeException();
	}
}

void SteamTurbineGovernor::initialize(Real Pref) {
	if (Pref>=0 && Pref<=1) {
		//Steady state at O equal to Om_ref (50Hz/60HZ)
    	mPref = Pref;
		mDelOm_prev = 0;
    	mDelOm = 0;
    	mDelPgv = 0;
		mPgv = Pref;
		mPgv_next = Pref;

    	SPDLOG_LOGGER_INFO(mSLog, 
			"Steam Governor initial values: \n"
			"\nPref: {:f}"
			"\nDelOm: {:f}"
			"\nDelPgv: {:f}"
			"\nPgv: {:f}",
			mPref, mDelOm, mDelPgv, mPgv);
	} else {
		SPDLOG_LOGGER_INFO(mSLog, 
			"\nP_ref of steam governor {} should be a value between 0 and 1 in pu",
			this->name());
		throw CPS::TypeException();
	}
}

Real SteamTurbineGovernor::step(Real Omega, Real dt) {
    // write the values that were calculated in the previous step
	mDelOm_prev = mDelOm;
	mPgv = mPgv_next;

	// Calculate the input of the governor for time step k
	 mDelOm = mParameters->OmRef-Omega;

	// Calculate thee input of integrator in PT1 via values of controller and output of governor 
	 mDelPgv = (1. / mParameters->R *( mDelOm + mParameters->T2 / dt * (mDelOm-mDelOm_prev)) + mPref- mPgv) * 1. / mParameters->T3;
	if (mDelPgv<mParameters->dPmin)
		mDelPgv = mParameters->dPmin;
	if(mDelPgv>mParameters->dPmax)
		mDelPgv = mParameters->dPmax;

	// Calculating uoutput of PT1 actuator, the output of the governor
	mPgv_next = dt * mDelPgv + mPgv;
	if(mPgv_next<mParameters->Pmin)
		mPgv_next = mParameters->Pmin;
	if(mPgv_next>mParameters->Pmax)
		mPgv_next = mParameters->Pmax;

	return mPgv;
}

    
