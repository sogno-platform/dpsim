/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_SolidStateTransformer.h>

using namespace CPS;

SP::Ph1::SolidStateTransformer::SolidStateTransformer(String uid, String name,
                                                      Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, false, false, logLevel),
      mPref(mAttributes->create<Real>("P_ref",
                                      std::numeric_limits<double>::infinity())),
      mQ1ref(mAttributes->create<Real>("Q1_ref")),
      mQ2ref(mAttributes->create<Real>("Q2_ref")) {
  SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", **mName, this->type());
  mSLog->flush();
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
  setTerminalNumber(2);
};

SimPowerComp<Complex>::Ptr SP::Ph1::SolidStateTransformer::clone(String name) {
  // everything set by initializeFromNodesAndTerminals
  return SolidStateTransformer::make(name, mLogLevel);
}

void SP::Ph1::SolidStateTransformer::setParameters(Real nomV1, Real nomV2,
                                                   Real Pref, Real Q1ref,
                                                   Real Q2ref) {
  mNominalVoltageEnd1 = nomV1;
  mNominalVoltageEnd2 = nomV2;
  **mPref = Pref;
  **mQ1ref = Q1ref;
  **mQ2ref = Q2ref;
  mP2 = -1 * std::sqrt(Pref * Pref + Q1ref * Q1ref - Q2ref * Q2ref);
}

void SP::Ph1::SolidStateTransformer::initializeFromNodesAndTerminals(
    Real frequency) {

  if (std::isinf(mP2)) {
    std::stringstream ss;
    ss << "SST >>" << this->name()
       << ": infinite or nan values. Or initialized before setting parameters.";
    throw std::invalid_argument(ss.str());
  }
  if ((**mPref * mP2) > 0) {
    throw std::invalid_argument(
        "power at primary and secondary sides should be opposite");
  }
  mSubLoadSide1 = Load::make(**mName + "_subLoad1", mLogLevel);
  mSubLoadSide1->setParameters(**mPref, **mQ1ref, mNominalVoltageEnd1);
  mSubLoadSide2 = Load::make(**mName + "_subLoad2", mLogLevel);
  mSubLoadSide2->setParameters(mP2, **mQ2ref, mNominalVoltageEnd2);
  mSubLoadSide1->connect({mTerminals[0]->node()});
  mSubLoadSide2->connect({mTerminals[1]->node()});

  addMNASubComponent(mSubLoadSide1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::NO_TASK, false);
  addMNASubComponent(mSubLoadSide2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::NO_TASK, false);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nTerminal 0 power flow: {:s} VA"
                     "\nTerminal 1 power flow: {:s} VA"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::complexToString(Complex(**mPref, **mQ1ref)),
                     Logger::complexToString(Complex(mP2, **mQ2ref)));
}

void SP::Ph1::SolidStateTransformer::calculatePerUnitParameters(
    Real baseApparentPower, Real baseOmega) {
  mPref_perUnit = **mPref / baseApparentPower;
  mP2_perUnit = mP2 / baseApparentPower;
  mQ1ref_perUnit = **mQ1ref / baseApparentPower;
  mQ2ref_perUnit = **mQ2ref / baseApparentPower;
  mSubLoadSide1->calculatePerUnitParameters(baseApparentPower, baseOmega);
  mSubLoadSide2->calculatePerUnitParameters(baseApparentPower, baseOmega);
  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n#### Calculate Per Unit Parameters for {}"
      "\nTerminal 0 power flow: {:s} p.u."
      "\nTerminal 1 power flow: {:s} p.u."
      "\n#### Calculate Per Unit Parameters finished ---",
      **mName, Logger::complexToString(Complex(mPref_perUnit, mQ1ref_perUnit)),
      Logger::complexToString(Complex(mP2_perUnit, mQ2ref_perUnit)));
}

Complex SP::Ph1::SolidStateTransformer::getNodalInjection(
    CPS::TopologicalNode::Ptr node) {
  if (node->name() == mTerminals[0]->node()->name()) {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\n#### get nodal injection for primary side"
        "\nreturned {:s} p.u.",
        Logger::complexToString(Complex(mPref_perUnit, mQ1ref_perUnit)));
    return Complex(mPref_perUnit, mQ1ref_perUnit);
  } else if (node->name() == mTerminals[1]->node()->name()) {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\n#### get nodal injection for secondary side"
        "\nreturned {:s} p.u.",
        Logger::complexToString(Complex(mP2_perUnit, mQ2ref_perUnit)));
    return Complex(mP2_perUnit, mQ2ref_perUnit);
  } else {
    throw std::invalid_argument(
        "Failed to process nodal power injection of Solid State Transformer" +
        this->name());
  }
}
