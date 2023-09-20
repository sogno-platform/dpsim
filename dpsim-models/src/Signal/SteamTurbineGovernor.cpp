#include <dpsim-models/Signal/SteamTurbineGovernor.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbineGovernor::SteamTurbineGovernor(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel){ }

void SteamTurbineGovernor::setParameters(Real OmRef, Real R, Real T2, Real T3,
                     Real dPmax, Real dPmin, Real Pmax, Real Pmin){
    
    mOmRef=OmRef;
    mR=R;
    mT2=T2;
    mT3=T3;
    mdPmax=dPmax;
    mdPmin=dPmin;
    mPmax=Pmax;
    mPmin=Pmin;

SPDLOG_LOGGER_INFO(mSLog, "SteamTurbineGovernor parameters: "
				"\nOmRef: {:e}"
				"\nR: {:e}"
				"\nT2: {:e}"
				"\nT3: {:e}"
				"\ndPmax: {:e}"
				"\ndPmin: {:e}"
				"\nPmax: {:e}"
				"\nPmin: {:e}",
				mOmRef, mR, mT2,
				mT3, mdPmax, mdPmin,
				mPmax, mPmin);
}

void SteamTurbineGovernor::initialize(Real Pref){
	if (Pref>=0 && Pref<=1){
	//Steady state at Om euqal to Om_ref (50Hz/60HZ)
    mPref=Pref;
	mDelOm_prev=0;
    mDelOm=0;
    mDelPgv=0;
	mPgv=Pref;
	mPgv_next=Pref;

    SPDLOG_LOGGER_INFO(mSLog, "Steam Governor initial values: \n"
				"\nPref: {:f}"
				"\nDelOm: {:f}"
				"\nDelPgv: {:f}"
				"\nPgv: {:f}",
				mPref,mDelOm,mDelPgv, mPgv);
	}
	else
	SPDLOG_LOGGER_INFO(mSLog, "P_ref should be a value between 0 and 1 in pu");
}

Real SteamTurbineGovernor::step(Real Omega, Real dt){
    //write the values that were calculated in the previous step
	mDelOm_prev=mDelOm;
	mPgv=mPgv_next;

	//Calctulate the input of the governor for time step k
	 mDelOm= mOmRef-Omega;

	//calculate thee input of integrator in PT1 via values of controller and output of governor 
	 mDelPgv= (1/mR *( mDelOm + mT2/dt*(mDelOm-mDelOm_prev)) + mPref- mPgv) * 1/mT3;
	if(mDelPgv<mdPmin)
	mDelPgv=mdPmin;
	if(mDelPgv>mdPmax)
	mDelPgv=mdPmax;

	//Caltulating uoutput of PT1 actuator, the output of the governor
	mPgv_next=dt*mDelPgv+mPgv;
	if(mPgv_next<mPmin)
	mPgv_next=mPmin;
	if(mPgv_next>mPmax)
	mPgv_next=mPmax;

	return mPgv;
}

    
