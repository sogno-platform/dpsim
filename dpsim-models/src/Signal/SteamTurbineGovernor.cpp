#include <dpsim-models/MathUtils.h>
#include <dpsim-models/Signal/SteamTurbineGovernor.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbineGovernor::SteamTurbineGovernor(const String &name,
                                                   CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel) {}

void SteamTurbineGovernor::setParameters(
    std::shared_ptr<Base::GovernorParameters> parameters) {

  if (auto params = std::dynamic_pointer_cast<Signal::SteamGorvernorParameters>(
          parameters)) {
    mParameters = params;
    if (mParameters->T2 == 0) {
      SPDLOG_LOGGER_ERROR(mSLog, "The parameter T2 can not be equal to zero!");
      throw CPS::Exception();
    }
    SPDLOG_LOGGER_INFO(mSLog,
                       "\nSteam Governor parameters:"
                       "\nOmRef: {:e}"
                       "\nR: {:e}"
                       "\nT2: {:e}"
                       "\nT3: {:e}"
                       "\ndPmax: {:e}"
                       "\ndPmin: {:e}"
                       "\nPmax: {:e}"
                       "\nPmin: {:e}"
                       "\nKbc: {:e}\n",
                       mParameters->OmRef, mParameters->R, mParameters->T1,
                       mParameters->T2, mParameters->T3, mParameters->dPmax,
                       mParameters->dPmin, mParameters->Pmax, mParameters->Pmin,
                       mParameters->Kbc);
    mSLog->flush();
  } else {
    std::cout << "Type of parameters class of " << this->name()
              << " has to be SteamGorvernorParameters!" << std::endl;
    throw CPS::TypeException();
  }
}

void SteamTurbineGovernor::initialize(Real Pref) {
  if (Pref >= 0 && Pref <= 1) {
    // Steady state at t=0 equal to Om_ref (50Hz/60HZ)
    mPref = Pref;
    mDelOm = 0;
    mDelOm_prev = 0;
    mDelOm_2prev = 0;
    mP1 = 0;
    mP1_prev = 0;
    mP = 0;
    mDerPgv = 0;
    mPgvLim = Pref;
    mPgv = Pref;

    if (mParameters->T1 == 0) {
      mCa = 0;
      mCb = 0;
    } else {
      mCa = mParameters->T2 / mParameters->T1;
      mCb = (mParameters->T1 - mParameters->T2) / mParameters->T1;
    }

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nSteam Governor initial values:"
                       "\nPref: {:f}"
                       "\nDelOm: {:f}"
                       "\nDelPgv: {:f}"
                       "\nPgv: {:f}",
                       mPref, mDelOm, mDerPgv, mPgv);
    mSLog->flush();
  } else {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\nP_ref of steam governor {} should be a value between 0 and 1 in pu",
        this->name());
    throw CPS::TypeException();
  }
}

Real SteamTurbineGovernor::step(Real Omega, Real dt) {

  // write the values that were calculated in the previous step
  mDelOm_2prev = mDelOm_prev;
  mDelOm_prev = mDelOm;
  mP1_prev = mP1;

  // Calculate the input of the governor for time step k
  mDelOm = mParameters->OmRef - Omega;

  // Transfer function 1/R (1+sT2)/(s+T1) = 1/R (T2/T1 + (T1-T2)/T1 *1/(1+sT1)) = P(s)/delOm(s)
  if (mParameters->T1 == 0) {
    mP = (1 / mParameters->R) *
         (mDelOm_prev + (mParameters->T2 / dt) * (mDelOm_prev - mDelOm_2prev));
  } else {
    mP1 = mP1_prev + (dt / mParameters->T1) * (mDelOm_prev * mCb - mP1_prev);
    mP = (1 / mParameters->R) * (mP1_prev + mDelOm_prev * mCa);
  }

  // Calculate the input of integrator
  mDerPgv = 1. / mParameters->T3 * (mPref + mP - mPgv);
  // TODO: use module of Pgv?
  if (mDerPgv < mParameters->dPmin)
    mDerPgv = mParameters->dPmin;
  if (mDerPgv > mParameters->dPmax)
    mDerPgv = mParameters->dPmax;
  mDerPgv = mDerPgv - mParameters->Kbc * (mPgvLim - mPgv);

  // Calculate PgvLim (before the limiter)
  // use Pgv instead?
  mPgvLim = mPgvLim + dt * mDerPgv;

  // Calculating output  of the governor
  if (mPgvLim < mParameters->Pmin)
    mPgv = mParameters->Pmin;
  else if (mPgvLim > mParameters->Pmax)
    mPgv = mParameters->Pmax;
  else
    mPgv = mPgvLim;

  return mPgv;
}
