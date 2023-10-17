#include <dpsim-models/Signal/SteamTurbine.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbine::SteamTurbine(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { }

void SteamTurbine::setParameters(std::shared_ptr<Base::TurbineParameters> parameters) {
    if (auto params = std::dynamic_pointer_cast<Signal::SteamTurbineParameters>(parameters)){
        mParameters = params;
		if (mParameters->Fhp + mParameters->Fip + mParameters->Flp != 1.0) {
            SPDLOG_LOGGER_INFO(mSLog, 
			    "Steam turbine parameters: "
			    "\nFhp: {:e}"
				"\nFip: {:e}"
				"\nFlp: {:e}"
				"\nTch: {:e}"
				"\nTrh: {:e}"
				"\nTco: {:e}",
				mParameters->Fhp, mParameters->Fip, mParameters->Flp,
				mParameters->Tch, mParameters->Trh, mParameters->Tco);
        } else{
            SPDLOG_LOGGER_INFO(mSLog, 
                "The sum of factors Fhp+Fip+Flp of the steam turbine {} should be equal to 1!", this->name());
            std::cout << "The sum of factors Fhp+Fip+Flp of the steam turbine " << this->name() << " should be equal to 1!" << std::endl;
            throw CPS::Exception();
        }
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be SteamTurbineParameters!" << std::endl;
		throw CPS::TypeException();
	}
}

void SteamTurbine::initialize(Real Pminit) {
    if (Pminit>=0 && Pminit<=1) {
        //steady state values
        mPhp = Pminit;
        mPip = Pminit;
        mPlp = Pminit;
        mPm = Pminit;
        mPhp_next = Pminit;
        mPip_next = Pminit;
        mPlp_next = Pminit;

        SPDLOG_LOGGER_INFO(mSLog, "Turbine initial values: \n"
			"\nPhp: {:f}"
			"\nPip: {:f}"
			"\nPlp: {:f}"
			"\nPm: {:f}",
			mPhp, mPip, mPlp, mPm);
    } else {
        SPDLOG_LOGGER_INFO(mSLog, "Initial mechanical power of steam turbine {} in pu should be betwenn 0 and 1", this->name());
        std::cout << "Initial mechanical power of steam turbine " << this->name() << " in pu should be betwenn 0 and 1" << std::endl;
        throw CPS::Exception();
    }
}

Real SteamTurbine::step(Real Pgv, Real dt) {
    // write old values in prev
    mPhp = mPhp_next;
    mPip = mPip_next;
    mPlp = mPlp_next;
    
    // Calculate next values of state variables of turbine with forward euler high pressure
    if(mParameters->Tch == 0)
        mPhp=Pgv;
    else
        mPhp_next= mPhp + dt/mParameters->Tch *(Pgv - mPhp);
   
    // intermidiat pressure 
    if (mParameters->Trh == 0)
        mPip = mPhp;
    else
        mPip_next=mPip + dt / mParameters->Trh * (mPhp - mPip);
    
    // lower pressure
    if (mParameters->Tco == 0)
        mPlp = mPip;
    else
        mPlp_next=mPlp + dt / mParameters->Tco *(mPip - mPlp);
    
    // Output
    mPm = mPhp * mParameters->Fhp + mPip * mParameters->Fip + mPlp * mParameters->Flp;
    
    // Mechanical power and torque in pu are equal
    return mPm;
}

