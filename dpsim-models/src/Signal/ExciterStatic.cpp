// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/Signal/ExciterStatic.h>

using namespace CPS;
using namespace CPS::Signal;

ExciterStatic::ExciterStatic(const String &name, CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel) {}

void ExciterStatic::setParameters(
    std::shared_ptr<Base::ExciterParameters> parameters) {

  if (auto params = std::dynamic_pointer_cast<Signal::ExciterStaticParameters>(
          parameters)) {
    mParameters = params;

    if (mParameters->Ta == mParameters->Tb) {
      SPDLOG_LOGGER_INFO(
          mSLog,
          "\n if Ta=Tb  the auxilary state variable Xb can be ignored, as Xb=0",
          this->name());
      throw CPS::TypeException();
    }

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nExciteStatic parameters:"
                       "\nTr: {:e}"
                       "\nTa: {:e}"
                       "\nTb: {:e}"
                       "\nTe: {:e}"
                       "\nKa: {:e}"
                       "\nMaximum EMF: {:e}"
                       "\nMinimum EMF: {:e}"
                       "\nKbc: {:e}\n",
                       mParameters->Tr, mParameters->Ta, mParameters->Tb,
                       mParameters->Te, mParameters->Ka, mParameters->MaxEfd,
                       mParameters->MinEfd, mParameters->Kbc);
  } else {
    SPDLOG_LOGGER_ERROR(
        mSLog, "Type of parameters class of {} has to be ExciterStatic!",
        this->name());
    throw CPS::TypeException();
  }
}

void ExciterStatic::initialize(Real Vh_init, Real Ef_init) {

  mVh = Vh_init;
  mEfd = Ef_init;
  mEfdLim = mEfd;

  if ((Ef_init > mParameters->MaxEfd) || (Ef_init < mParameters->MinEfd)) {
    SPDLOG_LOGGER_ERROR(mSLog,
                        "\nERROR: The initialisation is bad, Ef_init out of "
                        "allowed band ({}<Ef_init<{}).",
                        mParameters->MinEfd, mParameters->MaxEfd);
    throw CPS::Exception();
  }

  // initialize auxiliar parameters
  mCa = mParameters->Ta / mParameters->Tb;
  mCb = (mParameters->Tb - mParameters->Ta) / mParameters->Tb;

  /// init value of transducer output
  mVr = mVh;
  mVr_prev = mVr;

  // init value of the second lag block
  mVe = mEfd / mParameters->Ka;

  /// Input of the first lead lag block
  mVin = mVe / (mCa + mCb);

  /// Initial reference value
  mVref = mVin + mVr;

  /// Initial value for auxilar state variable
  mXb = mVin;
  mXb_prev = mXb;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nExciter Initialization"
                     "\nExciter type: ExciterStatic"
                     "\nInitially set excitation system initial values:"
                     "\ninit Vh: {:e}"
                     "\ninit Ef: {:e}"
                     "\nCalculated set poit and auxilary state variables:"
                     "\nVref : {:e}"
                     "\nXb : {:e}"
                     "\nVin = {:e}",
                     mVh, mEfd, mVref, mXb, mVin);
  mSLog->flush();
}

Real ExciterStatic::step(Real mVd, Real mVq, Real dt, Real Vpss) {

  // Voltage magnitude calculation
  mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));

  // update state variables at time k
  mVr_prev = mVr;
  mXb_prev = mXb;

  /// Compute Vr at time k
  if (mParameters->Tr > 0)
    mVr = mVr_prev + dt / mParameters->Tr * (mVh - mVr_prev);
  else
    mVr = mVh;

  /// Compute Vin at time k
  mVin = mVref - mVr_prev + Vpss;

  /// Compute Xb at time k+1 using euler forward
  mXb = mXb_prev + (mVin - mXb_prev) * dt / mParameters->Tb;

  // Compute Efd at time k using euler forward
  mVe = mVin * mCa + mXb_prev * mCb - mParameters->Kbc * (mEfd - mEfdLim);
  mEfd = mEfd + (dt / mParameters->Te) * (mParameters->Ka * mVe - mEfd);
  if (mEfd > mParameters->MaxEfd)
    mEfdLim = mParameters->MaxEfd;
  else if (mEfd < mParameters->MinEfd)
    mEfdLim = mParameters->MinEfd;
  else
    mEfdLim = mEfd;

  return mEfdLim;
}
