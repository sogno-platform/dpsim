#include <dpsim-models/Signal/SteamTurbine.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbine::SteamTurbine(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel){ }

void SteamTurbine::setParameters(Real Fhp, Real Fip, Real Flp, Real Tch, Real Trh, Real Tco){
    if (Fhp+Fip+Flp==1)
    {
    mFhp=Fhp;
    mFip=Fip;
    mFlp=Flp;
    mTch=Tch;
    mTrh=Trh;
    mTco=Tco;
    SPDLOG_LOGGER_INFO(mSLog, "SteamTurbine parameters: "
				"\nFhp: {:e}"
				"\nFip: {:e}"
				"\nFlp: {:e}"
				"\nTch: {:e}"
				"\nTrh: {:e}"
				"\nTco: {:e}",
				mFhp, mFip, mFlp,
				mTch, mTrh, mTco);
    }
    else{
           SPDLOG_LOGGER_INFO(mSLog, "The sum of factors Fhp+Fip+Flp should be equal to 1");
    }
}

void SteamTurbine::initialize(Real Pminit)
{
    if (Pminit>=0 && Pminit<=1)
    {
        //steady state values
        mPhp=Pminit;
        mPip=Pminit;
        mPlp=Pminit;
        mPm=Pminit;
        mPhp_next=Pminit;
        mPip_next=Pminit;
        mPlp_next=Pminit;

        SPDLOG_LOGGER_INFO(mSLog, "Turbine initial values: \n"
				"\nPhp: {:f}"
				"\nPip: {:f}"
				"\nPlp: {:f}"
				"\nPm: {:f}",
				mPhp, mPip, mPlp, mPm);
    }
    else
    SPDLOG_LOGGER_INFO(mSLog, "Initial mechanical power in pu should be betwenn 0 and 1");
}

    //Step depending on input from governing system and time step values. 
    //Mechanical power and toqueq in pu are equal
	Real SteamTurbine::step(Real Pgv, Real dt){

        //write old values in prev
        mPhp=mPhp_next;
        mPip=mPip_next;
        mPlp=mPlp_next;

        //Calculate next values of state variables of turbine with forward euler
        //high pressure
        if(mTch==0){
        mPhp=Pgv;
        }else
        {
        mPhp_next= mPhp + dt/mTch *(Pgv - mPhp);
        }
       
        //intermidiat pressure 
        if (mTrh==0){
        mPip=mPhp;
        } else{
        mPip_next=mPip + dt / mTrh *(mPhp - mPip);
        }
        //lower pressure
        if (mTco==0){
        mPlp=mPip;
        } else{
        mPlp_next=mPlp + dt / mTco *(mPip - mPlp);
        }
        
        //Output
        mPm=mPhp*mFhp+mPip*mFip+mPlp*mFlp;
        //Mechanical power and toqueq in pu are equal
        return mPm;
    }

