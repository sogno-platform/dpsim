#include <dpsim-models/MathUtils.h>
#include <dpsim-models/Signal/HydroTurbineGovernor.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::HydroTurbineGovernor::HydroTurbineGovernor(const String &name,
                                                   CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel) {}

void HydroTurbineGovernor::setParameters(
    std::shared_ptr<Base::GovernorParameters> parameters) {

  if (auto params = std::dynamic_pointer_cast<Signal::HydroGorvernorParameters>(
          parameters)) {
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
                       mParameters->T2, mParameters->T3, mParameters->Pmax,
                       mParameters->Pmin);
  } else {
    std::cout << "Type of parameters class of " << this->name()
              << " has to be HydroGorvernorParameters!" << std::endl;
    throw CPS::TypeException();
  }
  // FIXME: Ein problem entsteht bei der Zerlegung wenn T1=t3 (gleiche polstellen, es muss eine andere Zerlegung benutzt werden)
  if (mParameters->T1 == mParameters->T3) {
    SPDLOG_LOGGER_ERROR(mSLog, "T1 can not be equal to T3!!!");
    throw CPS::Exception();
  }

  mCa =
      (mParameters->T1 - mParameters->T2) / (mParameters->T1 - mParameters->T3);
  mCb =
      (mParameters->T2 - mParameters->T3) / (mParameters->T1 - mParameters->T3);
}

void HydroTurbineGovernor::initializeFromPowerFlow(Real Pref) {

  if (Pref >= mParameters->Pmin && Pref <= mParameters->Pmax) {
    mPgv = Pref;
    mPref = Pref;
    mDelOm = 0;
    mDelOm_prev = 0;
    mX1 = 0;
    mX1_prev = 0;
    mX2 = 0;
    mX2_prev = 0;
    SPDLOG_LOGGER_INFO(mSLog,
                       "Hydro Governor initial values: \n"
                       "\nPref: {:f}"
                       "\nDelOm: {:f}"
                       "\nX1: {:f}"
                       "\nX2: {:f}"
                       "\nPgv: {:f}",
                       Pref, mDelOm, mX1, mX2, mPgv);
  } else {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\nP_ref of hydro governor {} should be a value between 0 and 1 in pu",
        this->name());
    throw CPS::TypeException();
  }
}

Real HydroTurbineGovernor::step(Real Omega, Real dt) {
  // write the values that were calculated in the previous step
  mDelOm_prev = mDelOm;
  mX1_prev = mX1;
  mX2_prev = mX2;

  // Calculate the input of the governor for time step k
  mDelOm = mParameters->OmRef - Omega;

  // Calculation State variables for k+1 with integrators
  mX1 = mX1_prev + dt / mParameters->T1 * (mDelOm_prev - mX1_prev);
  mX2 = mX2_prev + dt / mParameters->T3 * (mDelOm_prev - mX2_prev);

  // Output of the governor before limiter, values from k are used to caltulate output Pgv(k)
  mPgv = mPref + 1. / mParameters->R * (mX1 * mCa + mX2 * mCb);

  if (mPgv > mParameters->Pmax)
    mPgv = mParameters->Pmax;
  if (mPgv < mParameters->Pmin)
    mPgv = mParameters->Pmin;

  return mPgv;
}
