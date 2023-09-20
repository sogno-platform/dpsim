#include <dpsim-models/Signal/HydroTurbineGovernor.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::HydroTurbineGovernor::HydroTurbineGovernor(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel){ }

void HydroTurbineGovernor::setParameters(Real OmRef, Real R, Real T1, Real T2, Real T3,
                                         Real Pmax, Real Pmin){
    
    mOmRef=OmRef;
    mR=R;
    mT1=T1;
    mT2=T2;
    mT3=T3;
    mPmax=Pmax;
    mPmin=Pmin;

SPDLOG_LOGGER_INFO(mSLog, "HydroTurbineGovernor parameters: "
				"\nOmRef: {:e}"
				"\nR: {:e}"
                "\nT1: {:e}"
				"\nT2: {:e}"
				"\nT3: {:e}"
				"\nPmax: {:e}"
				"\nPmin: {:e}",
				mOmRef, mR, mT1, mT2,
				mT3, mPmax, mPmin);
}

void HydroTurbineGovernor::initialize(Real Pref){

	if (Pref>=mPmin && Pref<=mPmax){
    mPgv=Pref;
    mPref=Pref;
    mDelOm = 0;
    mX1=0;
    mX1_next=0;
    mX2=0;
    mX2_next=0;
    SPDLOG_LOGGER_INFO(mSLog, "Hydro Governor initial values: \n"
				"\nPref: {:f}"
				"\nDelOm: {:f}"
				"\nX1: {:f}"
				"\nX2: {:f}"
				"\nPgv: {:f}",
				 Pref, mDelOm, mX1, mX2, mPgv);
	}
	else
	SPDLOG_LOGGER_INFO(mSLog, "P_ref should be a value between 0 and 1 in pu");
}

Real HydroTurbineGovernor::step(Real Omega, Real dt){
    //Ein problem entsteht bei dir Zerlegung wenn T1=t3 (gleiche polstellen, es muss eine andere Zerlegung benutzt werden)
    const Real cA = (mT1-mT2)/(mT1-mT3);
    const Real cB = (mT2-mT3)/(mT1-mT3);
    //Calciation of frequency deviation
    mDelOm=mOmRef-Omega;
    //Set the values of state variables calculated in last step as atual values for k
    mX1=mX1_next;
    mX2=mX2_next;
    //Caltulatin State variables for k+1 with integrators
    mX1_next= mX1 + dt/mT1 * (mDelOm-mX1);
    mX2_next= mX2 + dt/mT3 * (mDelOm-mX2);
    
    //Output of the governor before limiter, values from k are used to caltulate output Pgv(k)
    mPgv= mPref + 1/mR * ( mX1*cA + mX2 *cB);

    if (mPgv>mPmax)
    return mPmax;

    if(mPgv<mPmin)
    return mPmin;

	return mPgv;
}

    