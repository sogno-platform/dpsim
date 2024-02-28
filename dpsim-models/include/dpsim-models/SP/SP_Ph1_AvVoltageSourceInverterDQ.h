/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/Base/Base_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/SP/SP_Ph1_Capacitor.h>
#include <dpsim-models/SP/SP_Ph1_Inductor.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>
#include <dpsim-models/SP/SP_Ph1_Transformer.h>
#include <dpsim-models/SP/SP_Ph1_VoltageSource.h>
#include <dpsim-models/Signal/PLL.h>
#include <dpsim-models/Signal/PowerControllerVSI.h>

namespace CPS {
namespace SP {
namespace Ph1 {
class AvVoltageSourceInverterDQ
    : public CompositePowerComp<Complex>,
      public Base::AvVoltageSourceInverterDQ,
      public SharedFactory<AvVoltageSourceInverterDQ> {
protected:
  // ### General Parameters ###
  /// Nominal system angle
  Real mThetaN = 0;
  /// Simulation step
  Real mTimeStep;

  // ### Control Subcomponents ###
  /// PLL
  std::shared_ptr<Signal::PLL> mPLL;
  /// Power Controller
  std::shared_ptr<Signal::PowerControllerVSI> mPowerControllerVSI;

  // ### Electrical Subcomponents ###
  /// Controlled voltage source
  std::shared_ptr<SP::Ph1::VoltageSource> mSubCtrledVoltageSource;
  /// Resistor Rf as part of LCL filter
  std::shared_ptr<SP::Ph1::Resistor> mSubResistorF;
  /// Capacitor Cf as part of LCL filter
  std::shared_ptr<SP::Ph1::Capacitor> mSubCapacitorF;
  /// Inductor Lf as part of LCL filter
  std::shared_ptr<SP::Ph1::Inductor> mSubInductorF;
  /// Resistor Rc as part of LCL filter
  std::shared_ptr<SP::Ph1::Resistor> mSubResistorC;
  /// Optional connection transformer
  std::shared_ptr<SP::Ph1::Transformer> mConnectionTransformer;

  /// Flag for connection transformer usage
  Bool mWithConnectionTransformer = false;
  /// Flag for controller usage
  Bool mWithControl = true;

  // #### solver ####
  ///
  std::vector<const Matrix *> mRightVectorStamps;

public:
  // ### General Parameters ###
  /// Nominal frequency
  const Attribute<Real>::Ptr mOmegaN;
  /// Nominal voltage
  const Attribute<Real>::Ptr mVnom;
  /// Active power reference
  const Attribute<Real>::Ptr mPref;
  /// Reactive power reference
  const Attribute<Real>::Ptr mQref;

  // ### Inverter Interfacing Variables ###
  // Control inputs
  /// Measured voltage d-axis in local reference frame
  const Attribute<Real>::Ptr mVcd;
  /// Measured voltage q-axis in local reference frame
  const Attribute<Real>::Ptr mVcq;
  /// Measured current d-axis in local reference frame
  const Attribute<Real>::Ptr mIrcd;
  /// Measured current q-axis in local reference frame
  const Attribute<Real>::Ptr mIrcq;
  // Control outputs
  /// Voltage as control output after transformation interface
  const Attribute<MatrixComp>::Ptr mVsref;

  // Sub voltage source
  const Attribute<MatrixComp>::Ptr mVs;

  // PLL
  const Attribute<Matrix>::Ptr mPllOutput;

  // input, state and output vector for logging
  const Attribute<Matrix>::Ptr mPowerctrlInputs;
  const Attribute<Matrix>::Ptr mPowerctrlStates;
  const Attribute<Matrix>::Ptr mPowerctrlOutputs;

  /// Defines name amd logging level
  AvVoltageSourceInverterDQ(String name,
                            Logger::Level logLevel = Logger::Level::off)
      : AvVoltageSourceInverterDQ(name, name, logLevel) {}
  /// Defines UID, name, logging level and connection trafo existence
  AvVoltageSourceInverterDQ(String uid, String name,
                            Logger::Level logLevel = Logger::Level::off,
                            Bool withTrafo = false);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);
  /// Setter for general parameters of inverter
  void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref);
  /// Setter for parameters of control loops
  void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl,
                               Real Ki_powerCtrl, Real Kp_currCtrl,
                               Real Ki_currCtrl, Real Omega_cutoff);
  /// Setter for parameters of transformer
  void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
                                Real ratedPower, Real ratioAbs, Real ratioPhase,
                                Real resistance, Real inductance);
  /// Setter for parameters of filter
  void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
  /// Setter for initial values applied in controllers
  void setInitialStateValues(Real pInit, Real qInit, Real phi_dInit,
                             Real phi_qInit, Real gamma_dInit,
                             Real gamma_qInit);
  void withControl(Bool controlOn) { mWithControl = controlOn; };

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  /// Updates current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates voltage across component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  // #### Control section ####
  /// Control pre step operations
  void controlPreStep(Real time, Int timeStepCount);
  /// Perform step of controller
  void controlStep(Real time, Int timeStepCount);
  /// Add control step dependencies
  void
  addControlPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                AttributeBase::List &attributeDependencies,
                                AttributeBase::List &modifiedAttributes) const;
  /// Add control step dependencies
  void
  addControlStepDependencies(AttributeBase::List &prevStepDependencies,
                             AttributeBase::List &attributeDependencies,
                             AttributeBase::List &modifiedAttributes) const;

  class ControlPreStep : public CPS::Task {
  public:
    ControlPreStep(AvVoltageSourceInverterDQ &AvVoltageSourceInverterDQ)
        : Task(**AvVoltageSourceInverterDQ.mName + ".ControlPreStep"),
          mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
      mAvVoltageSourceInverterDQ.addControlPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mAvVoltageSourceInverterDQ.controlPreStep(time, timeStepCount);
    };

  private:
    AvVoltageSourceInverterDQ &mAvVoltageSourceInverterDQ;
  };

  class ControlStep : public CPS::Task {
  public:
    ControlStep(AvVoltageSourceInverterDQ &AvVoltageSourceInverterDQ)
        : Task(**AvVoltageSourceInverterDQ.mName + ".ControlStep"),
          mAvVoltageSourceInverterDQ(AvVoltageSourceInverterDQ) {
      mAvVoltageSourceInverterDQ.addControlStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mAvVoltageSourceInverterDQ.controlStep(time, timeStepCount);
    };

  private:
    AvVoltageSourceInverterDQ &mAvVoltageSourceInverterDQ;
  };
};
} // namespace Ph1
} // namespace SP
} // namespace CPS