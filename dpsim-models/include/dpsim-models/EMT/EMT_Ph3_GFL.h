// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_VoltageSource.h>
#include <dpsim-models/Signal/PLL.h>
#include <dpsim-models/Signal/PowerControllerVSI.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Grid-following voltage-source inverter with explicit EMT filter.
///
/// Electrical topology:
///
///   controlled voltage source -- Rf -- Lf -- PCC
///                                      |
///                                      Cf
///                                      |
///                                     GND
///
/// The filter capacitor is connected directly at the external PCC.
/// There is intentionally:
///   - no connection transformer,
///   - no coupling resistor Rc.
///
/// Controller measurements:
///   - voltage: PCC / capacitor voltage,
///   - current: positive converter-to-grid current.
///
/// DPsim's component interface-current convention is retained at mIntfCurrent:
/// positive mIntfCurrent means current from the external network into the
/// component. Therefore the controller explicitly uses -mIntfCurrent.
class GFL final : public CompositePowerComp<Real>, public SharedFactory<GFL> {
protected:
  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  Real mOmegaN = 0.0;
  Real mVnom = 0.0;
  Real mPRef = 0.0;
  Real mQRef = 0.0;

  Real mLf = 0.0;
  Real mCf = 0.0;
  Real mRf = 0.0;

  Real mKpPLL = 0.0;
  Real mKiPLL = 0.0;
  Real mKpPowerCtrl = 0.0;
  Real mKiPowerCtrl = 0.0;
  Real mKpCurrCtrl = 0.0;
  Real mKiCurrCtrl = 0.0;
  Real mOmegaCutoff = 0.0;

  Real mTimeStep = 0.0;

  Bool mWithControl = true;
  Bool mFilterParametersSet = false;
  Bool mControllerParametersSet = false;
  Bool mManualInitialStatesSet = false;

  Real mPInitManual = 0.0;
  Real mQInitManual = 0.0;
  Real mPhiDInitManual = 0.0;
  Real mPhiQInitManual = 0.0;
  Real mGammaDInitManual = 0.0;
  Real mGammaQInitManual = 0.0;

  // -------------------------------------------------------------------------
  // Control subcomponents
  // -------------------------------------------------------------------------
  std::shared_ptr<Signal::PLL> mPLL;
  std::shared_ptr<Signal::PowerControllerVSI> mPowerControllerVSI;

  // -------------------------------------------------------------------------
  // Electrical subcomponents
  // -------------------------------------------------------------------------
  std::shared_ptr<EMT::Ph3::VoltageSource> mSubControlledVoltageSource;
  std::shared_ptr<EMT::Ph3::Resistor> mSubResistorF;
  std::shared_ptr<EMT::Ph3::Inductor> mSubInductorF;
  std::shared_ptr<EMT::Ph3::Capacitor> mSubCapacitorF;

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------
  Matrix getParkTransformMatrixPowerInvariant(Real theta) const;
  Matrix getInverseParkTransformMatrixPowerInvariant(Real theta) const;

  Matrix parkTransformPowerInvariant(Real theta, const Matrix &fabc) const;
  Matrix inverseParkTransformPowerInvariant(Real theta,
                                            const Matrix &fdq) const;

  void updateMeasurementAttributes(Real theta);
  void updatePowerAndFrequencyAttributes();

public:
  using SharedFactory<GFL>::make;

  // -------------------------------------------------------------------------
  // Public attributes
  // -------------------------------------------------------------------------

  /// PCC voltage in controller dq frame.
  const Attribute<Real>::Ptr mVcd;
  const Attribute<Real>::Ptr mVcq;

  /// Positive converter-to-grid current in controller dq frame.
  const Attribute<Real>::Ptr mIgridD;
  const Attribute<Real>::Ptr mIgridQ;

  /// Instantaneous controller-frame active/reactive power.
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;

  /// Estimated PLL angular frequency [rad/s].
  const Attribute<Real>::Ptr mOmegaPLL;

  /// Controlled source voltage reference in abc instantaneous peak values.
  const Attribute<Matrix>::Ptr mVsref;

  /// Actual controlled source voltage.
  const Attribute<Matrix>::Ptr mVs;

  /// PLL output [theta, phi_pll].
  const Attribute<Matrix>::Ptr mPllOutput;

  /// Power-controller logging vectors.
  const Attribute<Matrix>::Ptr mPowerctrlInputs;
  const Attribute<Matrix>::Ptr mPowerctrlOutputs;
  const Attribute<Matrix>::Ptr mPowerctrlStates;

  // -------------------------------------------------------------------------
  // Construction
  // -------------------------------------------------------------------------
  GFL(String uid, String name, Logger::Level logLevel = Logger::Level::off);

  GFL(String name, Logger::Level logLevel = Logger::Level::off)
      : GFL(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------

  /// General operating parameters.
  void setParameters(Real sysOmega, Real sysVoltNom, Real pRef, Real qRef);

  /// Controller parameters.
  void setControllerParameters(Real kpPLL, Real kiPLL, Real kpPowerCtrl,
                               Real kiPowerCtrl, Real kpCurrCtrl,
                               Real kiCurrCtrl, Real omegaCutoff);

  /// Filter parameters. No Rc exists in this model.
  void setFilterParameters(Real lf, Real cf, Real rf);

  /// Optional manual controller-state initialization.
  ///
  /// If this is not called, the class computes a steady-state initialization
  /// automatically from the PCC voltage and requested P/Q.
  void setInitialStateValues(Real pInit, Real qInit, Real phiDInit,
                             Real phiQInit, Real gammaDInit, Real gammaQInit);

  void withControl(Bool controlOn) { mWithControl = controlOn; }

  // -------------------------------------------------------------------------
  // Initialization
  // -------------------------------------------------------------------------
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  // -------------------------------------------------------------------------
  // MNA
  // -------------------------------------------------------------------------
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;

  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

  void mnaParentPreStep(Real time, Int timeStepCount) override;

  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;

  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;

  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  // -------------------------------------------------------------------------
  // Control
  // -------------------------------------------------------------------------
  void controlPreStep(Real time, Int timeStepCount);
  void controlStep(Real time, Int timeStepCount);

  void addControlPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                     AttributeBase::List &attributeDependencies,
                                     AttributeBase::List &modifiedAttributes);

  void addControlStepDependencies(AttributeBase::List &prevStepDependencies,
                                  AttributeBase::List &attributeDependencies,
                                  AttributeBase::List &modifiedAttributes);

  class ControlPreStep : public CPS::Task {
  public:
    explicit ControlPreStep(GFL &gfl)
        : Task(**gfl.mName + ".ControlPreStep"), mGFL(gfl) {
      mGFL.addControlPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }

    void execute(Real time, Int timeStepCount) override {
      mGFL.controlPreStep(time, timeStepCount);
    }

  private:
    GFL &mGFL;
  };

  class ControlStep : public CPS::Task {
  public:
    explicit ControlStep(GFL &gfl)
        : Task(**gfl.mName + ".ControlStep"), mGFL(gfl) {
      mGFL.addControlStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }

    void execute(Real time, Int timeStepCount) override {
      mGFL.controlStep(time, timeStepCount);
    }

  private:
    GFL &mGFL;
  };
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
