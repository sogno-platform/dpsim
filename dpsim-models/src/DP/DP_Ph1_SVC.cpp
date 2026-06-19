/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SVC.h>

using namespace CPS;

DP::Ph1::SVC::SVC(String uid, String name, Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mVpcc(mAttributes->create<Real>("Vpcc", 0)),
      mVmeasPrev(mAttributes->create<Real>("Vmeas", 0)) {
  setTerminalNumber(1);
  setVirtualNodeNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  mDeltaV = mAttributes->create<Real>("DeltaV", 0);
  mBPrev = mAttributes->create<Real>("B");
  mViolationCounter = mAttributes->create<Real>("ViolationCounter", 0);
}

Bool DP::Ph1::SVC::ValueChanged() { return mValueChange; }

void DP::Ph1::SVC::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  // Inductor/capacitor values depend on omega computed from the simulation
  // frequency in initializeParentFromNodesAndTerminals(). They are created
  // here (topology) and parametrized there (values).

  // Inductor with Switch
  mSubInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->connect({SimNode::GND, mVirtualNodes[0]});
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductorSwitch =
      std::make_shared<DP::Ph1::Switch>(**mName + "_Lswitch", mLogLevel);
  mSubInductorSwitch->setParameters(mSwitchROpen, mSwitchRClosed, false);
  mSubInductorSwitch->connect({mVirtualNodes[0], mTerminals[0]->node()});
  addMNASubComponent(mSubInductorSwitch,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  // Capacitor with Switch
  mSubCapacitor =
      std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
  mSubCapacitor->connect({SimNode::GND, mVirtualNodes[1]});
  addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubCapacitorSwitch =
      std::make_shared<DP::Ph1::Switch>(**mName + "_Cswitch", mLogLevel);
  mSubCapacitorSwitch->setParameters(mSwitchROpen, mSwitchRClosed, false);
  mSubCapacitorSwitch->connect({mVirtualNodes[1], mTerminals[0]->node()});
  addMNASubComponent(mSubCapacitorSwitch,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
}

void DP::Ph1::SVC::initializeParentFromNodesAndTerminals(Real frequency) {
  // initial state is both switches are open
  Real omega = 2. * PI * frequency;
  // init L and C with small/high values (both have high impedance)
  Real LInit = 1e6 / omega;
  Real CInit = 1e-6 / omega;
  mLPrev = LInit;
  mCPrev = CInit;

  // impedances of both branches
  Complex LImpedance = {mSwitchROpen, omega * LInit};
  Complex CImpedance = {mSwitchROpen, -1 / (omega * CInit)};
  Complex impedance = LImpedance * CImpedance / (LImpedance + CImpedance);

  (**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

  **mBPrev = 0;
  mPrevVoltage = (**mIntfVoltage)(0, 0).real();
  **mVmeasPrev = mPrevVoltage;

  if (mMechMode) {
    SPDLOG_LOGGER_INFO(mSLog, "Using Mechanical Model");
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n --- Parameters ---"
                     "\n Controller: T = {} K = {}"
                     "\n Reference Voltage  {} [kV]"
                     "\n Qmax = {} [var] -> BN = {} [S]"
                     "\n Bmax = {} Bmin = {} [p.u.]"
                     "\n Initial B: {}",
                     mTr, mKr, mRefVolt, mQN, mBN, mBMax, mBMin, **mBPrev);

  // set voltages at virtual nodes
  Complex VLSwitch =
      (**mIntfVoltage)(0, 0) - LImpedance * (**mIntfCurrent)(0, 0);
  mVirtualNodes[0]->setInitialVoltage(VLSwitch);
  Complex VCSwitch =
      (**mIntfVoltage)(0, 0) - CImpedance * (**mIntfCurrent)(0, 0);
  mVirtualNodes[1]->setInitialVoltage(VCSwitch);

  mSubInductor->setParameters(LInit);
  mSubCapacitor->setParameters(CInit);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nImpedance: {}"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     impedance, Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)));
}

// #### MNA functions ####

void DP::Ph1::SVC::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::SVC::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);

  if (time > 0.1 && !mDisconnect) {
    if (mMechMode) {
      mechanicalModelUpdateSusceptance(time);
    } else {
      updateSusceptance();
    }
    checkProtection(time);
  }
}

void DP::Ph1::SVC::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::SVC::mnaParentPostStep(Real time, Int timeStepCount,
                                     Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);

  mDeltaT = time - mPrevTimeStep;
  mPrevTimeStep = time;
  mValueChange = false;
}

void DP::Ph1::SVC::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mVpcc = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0),
                                           mNumFreqs, 0)
                .real();
  (**mIntfVoltage)(0, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::SVC::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = 0;
  (**mIntfCurrent)(0, 0) += mSubInductor->intfCurrent()(0, 0);
  (**mIntfCurrent)(0, 0) += mSubCapacitor->intfCurrent()(0, 0);
}

void DP::Ph1::SVC::checkProtection(Real time) {
  // check states for violation of protection values
  // get inverse protection curve value (time delay value)

  Real Vpu = **mVmeasPrev / mNomVolt;
  if (Vpu > 1.4) {
    mProtCount1 = mProtCount1 + mDeltaT;
    if (mProtCount1 > 0.1) {
      mDisconnect = true;
    }
  } else {
    mProtCount1 = 0;
  }
  if (Vpu > 1.25) {
    mProtCount2 = mProtCount2 + mDeltaT;
    if (mProtCount2 > 1) {
      mDisconnect = true;
    }
  } else {
    mProtCount2 = 0;
  }
  if (Vpu > 1.15) {
    mProtCount3 = mProtCount3 + mDeltaT;
    if (mProtCount3 > 5) {
      mDisconnect = true;
    }
  } else {
    mProtCount3 = 0;
  }

  if (mDisconnect) {
    SPDLOG_LOGGER_INFO(mSLog, "Disconnect SVC because of overvoltage at {}",
                       time);
    mSubCapacitorSwitch->open();
    mSubInductorSwitch->open();
    mValueChange = true;
  }
}

void DP::Ph1::SVC::updateSusceptance() {
  // calculate new B value
  // summarize some constants
  Real Fac1 = mDeltaT / (2 * mTr);
  Real Fac2 = mDeltaT * mKr / (2 * mTr);

  Real V = Math::abs((**mIntfVoltage)(0, 0).real());

  // Pt1 with trapez rule for voltage measurement
  Real Fac3 = mDeltaT / (2 * mTm);
  Real Vmeas = (1 / (1 + Fac3)) * (V + mPrevVoltage - **mVmeasPrev);

  **mDeltaV = (Vmeas - mRefVolt) / mNomVolt;
  Real deltaVPrev = (**mVmeasPrev - mRefVolt) / mNomVolt;

  // calc new B with trapezoidal rule
  Real B = (1 / (1 + Fac1)) *
           (Fac2 * (**mDeltaV + deltaVPrev) + (1 - Fac1) * **mBPrev);

  // check bounds
  if (B > mBMax) {
    B = mBMax;
  } else if (B < mBMin) {
    B = mBMin;
  }

  // set new B if it has a new value and difference is big enough
  if (B != **mBPrev) {
    Real omega = 2 * M_PI * mFrequencies(0, 0);

    if (B > 0) {
      // model inductive behaviour (decrease voltage)
      Real inductance = 1 / (omega * B * mBN);
      //check if change in reactance is sufficient to trigger a change
      if (Math::abs(1 - inductance / mLPrev) > 0.01) {
        mInductiveMode = true;
        mSubInductor->updateInductance(inductance, mDeltaT);
        mLPrev = inductance;

        mValueChange = true;
        mBSetCounter = 0;
      }
    } else {
      // model capacitive behaviour (increase voltage)
      Real capacitance = B * mBN / (-omega);
      //check if change in reactance is sufficient to trigger a change
      if (Math::abs(1 - capacitance / mCPrev) > 0.01) {
        mInductiveMode = false;
        mSubCapacitor->updateCapacitance(capacitance, mDeltaT);
        mCPrev = capacitance;

        mValueChange = true;
        mBSetCounter = 0;
      }
    }

    // update inductance model
    setSwitchState();
  } else {
    mBSetCounter = mBSetCounter + mDeltaT;
  }

  // save values
  **mBPrev = B;
  mPrevVoltage = V;
  **mVmeasPrev = Vmeas;
}

// model SVC with a mechanical component and discrete
void DP::Ph1::SVC::mechanicalModelUpdateSusceptance(Real time) {
  // current voltage
  Real V = Math::abs((**mIntfVoltage)(0, 0).real());
  Real omega = 2 * M_PI * mFrequencies(0, 0);

  // Pt1 with trapez rule for voltage measurement
  Real Fac3 = mDeltaT / (2 * mTm);
  Real Vmeas = (1 / (1 + Fac3)) * (V + mPrevVoltage - **mVmeasPrev);

  // V diff in pu
  Real deltaV = (mRefVolt - Vmeas) / mRefVolt;

  if (Math::abs(deltaV) > mDeadband) {
    if (**mViolationCounter > mMechSwitchDelay) {
      // change suszeptance one step
      if (deltaV > 0 && (mTapPos > mMinPos)) {
        // undervoltage

        mTapPos = mTapPos - 1;
        mTapPos = (mTapPos < mMinPos) ? mMinPos : mTapPos;
        **mViolationCounter = 0;
        SPDLOG_LOGGER_INFO(mSLog,
                           "Time: {}"
                           "\nDecreasing Tap. Reason: Undervoltage"
                           "\nNew Tap Position: {}",
                           time, mTapPos);
      } else if (deltaV < 0 && (mTapPos < mMaxPos)) {
        // overvoltage
        mTapPos = mTapPos + 1;
        mTapPos = (mTapPos > mMaxPos) ? mMaxPos : mTapPos;
        **mViolationCounter = 0;
        SPDLOG_LOGGER_INFO(mSLog,
                           "Time: {}"
                           "\nIncreasing Tap. Reason: Overvoltag"
                           "\nNew Tap Position: {}",
                           time, mTapPos);
      }

      if (**mViolationCounter == 0) {
        // new value for suszeptance
        if (mTapPos > 0) {
          // inductor is active
          mInductiveMode = true;
          Real inductance = 1 / ((mTapPos / mMaxPos) * mBN * omega);
          SPDLOG_LOGGER_INFO(mSLog, "New inductance: {}", inductance);
          mSubInductor->updateInductance(inductance, mDeltaT);
          mValueChange = true;
          setSwitchState();
        } else if (mTapPos < 0) {
          // capacitor is active
          mInductiveMode = false;
          Real capacitance = ((mTapPos / mMinPos) * mBN) / omega;
          SPDLOG_LOGGER_INFO(mSLog, "New capacitance: {}", capacitance);
          mSubCapacitor->updateCapacitance(capacitance, mDeltaT);
          mValueChange = true;
          setSwitchState();

        } else if (mTapPos = 0) {
          // open both
          SPDLOG_LOGGER_INFO(mSLog,
                             "Time: {}"
                             "Tap Position: 0. Open both elements",
                             time);
          mSubInductorSwitch->open();
          mSubCapacitorSwitch->open();
        }
      }
    } else {
      // increase counter
      **mViolationCounter = **mViolationCounter + mDeltaT;
    }
  } else {
    // reset counter
    **mViolationCounter = 0;
  }

  // save states
  mPrevVoltage = V;
  **mVmeasPrev = Vmeas;
}

void DP::Ph1::SVC::setSwitchState() {
  // set switches according to current mode of svc
  if (mInductiveMode) {
    if (!mSubInductorSwitch->mnaIsClosed()) {
      SPDLOG_LOGGER_INFO(mSLog, "Inductive Mode: Closed Inductor Switch");
      mSubInductorSwitch->close();
    }
    if (mSubCapacitorSwitch->mnaIsClosed()) {
      mSubCapacitorSwitch->open();
      SPDLOG_LOGGER_INFO(mSLog, "Inductive Mode: Opened Capacitor Switch");
    }
  } else {
    if (mSubInductorSwitch->mnaIsClosed()) {
      mSubInductorSwitch->open();
      SPDLOG_LOGGER_INFO(mSLog, "Capacitive Mode: Openend Inductor Switch");
    }
    if (!mSubCapacitorSwitch->mnaIsClosed()) {
      mSubCapacitorSwitch->close();
      SPDLOG_LOGGER_INFO(mSLog, "Capacitive Mode: Closed Capcitor Switch");
    }
  }
}
