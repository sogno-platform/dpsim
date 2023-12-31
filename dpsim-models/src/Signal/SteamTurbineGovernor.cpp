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
				"\nSteam Governor parameters:"
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
			mSLog->flush();
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be SteamGorvernorParameters!" << std::endl;
		throw CPS::TypeException();
	}
}

void SteamTurbineGovernor::initialize(Real Pref) {
	if (Pref>=0 && Pref<=1) {
		// Steady state at O equal to Om_ref (50Hz/60HZ)
    	mPref = Pref;
		mDelOm_prev = 0;
    	mDelOm = 0;
    	mDelPgv = 0;
		mPgv = Pref;
		mPgv_next = Pref;
		mPlim_in = Pref;
		mPlim_in_next = Pref;
		mP1 = 0;
		mP1_next = 0;
		mP = 0;

    	SPDLOG_LOGGER_INFO(mSLog, 
			"\nSteam Governor initial values:"
			"\nPref: {:f}"
			"\nDelOm: {:f}"
			"\nDelPgv: {:f}"
			"\nPgv: {:f}",
			mPref, mDelOm, mDelPgv, mPgv);
		mSLog->flush();
	} else {
		SPDLOG_LOGGER_INFO(mSLog, 
			"\nP_ref of steam governor {} should be a value between 0 and 1 in pu",
			this->name());
		throw CPS::TypeException();
	}
}

Real SteamTurbineGovernor::step(Real Omega, Real dt) {

	// WIndup is implemented to compare the models to IAEW. If no windup is desired set Kbc=0
	const Real cKbc=10;

	const Real cA=mParameters->T2/mParameters->T1;
	const Real cB=(mParameters->T1 - mParameters->T2) / mParameters->T1;

    // write the values that were calculated in the previous step
	mDelOm_prev = mDelOm;
	mPgv = mPgv_next;
	mPlim_in = mPlim_in_next;
	mP1 = mP1_next;

	// Calculate the input of the governor for time step k
	 mDelOm = mParameters->OmRef-Omega;

	// Transfer function 1/R (1+sT2)/(s+T1) = 1/R (T2/T1 + (T1-T2)/T1 *1/(1+sT1)) = P(s)/delOm(s)
 	if(mParameters->T1==0) {
		mP = (1/mParameters->R) * (mDelOm + (mParameters->T2/dt) * (mDelOm-mDelOm_prev));
	} else {
		mP1_next = mP1+(dt/mParameters->T1) * (mDelOm*cB-mP1);
		mP = (1/mParameters->R) * (mP1+mDelOm * cA);
	}

	// Calculate thee input of integrator in PT1 via values of controller and output of governor
	mDelPgv = (mPref + mP - mPgv) / mParameters->T3;
	if (mDelPgv<mParameters->dPmin)
		mDelPgv = mParameters->dPmin;
	if(mDelPgv>mParameters->dPmax)
		mDelPgv = mParameters->dPmax;

	// Calculating output of PT1 actuator, the output of the governor, without windup
	/*
	mPgv_next = dt * mDelPgv + mPgv;
	if(mPgv_next<mParameters->Pmin)
		mPgv_next = mParameters->Pmin;
	if(mPgv_next>mParameters->Pmax)
		mPgv_next = mParameters->Pmax; */

	// Calculating output of PT1 actuator, the output of the governor, including windup
	
	mPlim_in_next = dt *(mDelPgv - cKbc*(mPlim_in - mPgv)) + mPlim_in;
	if(mPlim_in_next<mParameters->Pmin){
		mPgv_next = mParameters->Pmin;
	}
	else if(mPlim_in_next>mParameters->Pmax){
		mPgv_next = mParameters->Pmax;
	}
	else{
		mPgv_next=mPlim_in_next;
	}

	return mPgv;
}

    
