#include <dpsim-models/Signal/HydroTurbine.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::HydroTurbine::HydroTurbine(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { }

void HydroTurbine::setParameters(std::shared_ptr<Base::TurbineParameters> parameters) {
    if (auto params = std::dynamic_pointer_cast<Signal::HydroTurbineParameters>(parameters)){
        mParameters = params;
        SPDLOG_LOGGER_INFO(mSLog, 
            "HydroTurbine parameters: "
			"\nTw: {:e}", mParameters->Tw);
    } else {
		std::cout << "Type of parameters class of " << this->name() << " has to be HydroTurbineParameters!" << std::endl;
		throw CPS::TypeException();
	}
}

void HydroTurbine::initialize(Real Pminit) {
    if (Pminit >= 0 && Pminit <= 1) {
        //steady state values
        mPm = Pminit;
        mX1 = Pminit;
        mX1_next = Pminit;
   
        SPDLOG_LOGGER_INFO(mSLog, "Hydro Turbine initial values: \n"
				"\nPm: {:f}"
				"\nX1: {:f}"
				"\nPlp: {:f}"
				"\nPm: {:f}",
				mPm, mX1);
    }
    else
        SPDLOG_LOGGER_INFO(mSLog, "Initial mechanical power of hydro turbine {} in pu should be betwenn 0 and 1", this->name());
        std::cout << "Initial mechanical power of hydro turbine " << this->name() << " in pu should be betwenn 0 and 1" << std::endl;
        throw CPS::Exception();
    }
 
Real HydroTurbine::step(Real Pgv, Real dt){
    // the values pre calculated in the previous step are now actual values
    mX1 = mX1_next;
    // Caltulating the value of the only state variable for the next step (integration) using the current
    mX1_next = 2 * dt/mParameters->Tw * (Pgv-mX1) + mX1;
    // Output is the value Pm(k) = -2*Pgv(k)+X1(k) 
    mPm = 3 * mX1 - (2 * Pgv);
    return mPm;
}
