// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/Signal/ExciterST1Simp.h>

using namespace CPS;

Signal::ExciterST1Simp::ExciterST1Simp(const String &name,
                                       CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel) {}

void Signal::ExciterST1Simp::setParameters(
    std::shared_ptr<Base::ExciterParameters> parameters) {

  if (auto params =
          std::dynamic_pointer_cast<Signal::ExciterST1Parameters>(parameters)) {
    mParameters = params;

    SPDLOG_LOGGER_INFO(mSLog,
                       "Exciter ST1Simp parameters:"
                       "\nTr: {:e}"
                       "\nKa: {:e}\n",
                       mParameters->Tr, mParameters->Ka);
  } else {
    std::cout << "Type of parameters class of " << this->name()
              << " has to be ExciterST1Simp!" << std::endl;
    throw CPS::TypeException();
  }
}

void Signal::ExciterST1Simp::initialize(Real Vh_init, Real Ef_init) {

  mVh = Vh_init;
  mEf = Ef_init;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Initially set excitation system initial values:"
                     "\ninit Vh: {:e}"
                     "\ninit Ef: {:e}",
                     mVh, mEf);

  /// init value of transducer output
  mVr = mVh;

  /// exciter reference
  mVref = mVr - mEf / mParameters->Ka;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Actually applied excitation system initial values:"
                     "\nVref : {:e}"
                     "\ninit Vr: {:e}",
                     mVref, mVr);
  mSLog->flush();
}

Real Signal::ExciterST1Simp::step(Real Vd, Real Vq, Real dt, Real Vpss) {
  // Voltage magnitude calculation
  mVh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));

  // update state variables at time k-1
  mVr_prev = mVr;

  // compute state variables at time k using euler forward

  // Voltage Transducer
  mVr = mVr_prev + dt / mParameters->Tr * (mVh - mVr_prev);

  // Exciter output
  mEf = (mVr + Vpss - mVref) * mParameters->Ka;

  return mEf;
}
