// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#pragma once
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/EMT/EMT_Ph3_ControlledCurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// @brief One end of a Bergeron travelling-wave line. Two of these replace a
/// single Signal::DecouplingLineEMT_Ph3 and can live in different systems or
/// different simulators, since each only ever reads far-end quantities that are
/// one travel time old.
class HalfDecouplingLine : public CompositePowerComp<Real>,
                           public SharedFactory<HalfDecouplingLine> {
protected:
  /// Travel time of the whole line
  Real mDelay;
  Matrix mResistance = Matrix::Zero(3, 3);
  Matrix mInductance = Matrix::Zero(3, 3);
  Matrix mCapacitance = Matrix::Zero(3, 3);
  /// Surge impedance, sqrt(L/C)
  Matrix mSurgeImpedance = Matrix::Zero(3, 3);

  // ### Electrical Subcomponents ###
  /// Controlled current source carrying the history term
  std::shared_ptr<EMT::Ph3::ControlledCurrentSource> mSubCtrledCurrentSource;
  /// Terminating impedance calculated from line parameters
  std::shared_ptr<EMT::Ph3::Resistor> mSubRes;

  /// Ring buffers for the values of previous timesteps, one row per stored step
  Matrix mVoltBuf, mCurBuf;
  UInt mBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;

  /// Reads a ring buffer one travel time back
  Matrix interpolate(Matrix &data);

public:
  typedef std::shared_ptr<HalfDecouplingLine> Ptr;

  /// History current fed into the controlled current source
  const Attribute<Matrix>::Ptr mSrcCtrledCurrent;
  /// Terminating resistance
  const Attribute<Matrix>::Ptr mSrcRes;
  /// Far-end voltage one travel time ago, supplied by the other half
  const Attribute<Matrix>::Ptr mReceivingVolt;
  /// Far-end current one travel time ago, supplied by the other half
  const Attribute<Matrix>::Ptr mReceivingCur;
  /// This end's voltage one travel time ago, for the other half to read
  const Attribute<Matrix>::Ptr mSendingVolt;
  /// This end's current one travel time ago, for the other half to read
  const Attribute<Matrix>::Ptr mSendingCur;

  /// Defines name and logging level
  HalfDecouplingLine(String name, Logger::Level logLevel = Logger::Level::off)
      : HalfDecouplingLine(name, name, logLevel) {}
  /// Defines UID, name and logging level
  HalfDecouplingLine(String uid, String name,
                     Logger::Level logLevel = Logger::Level::off);

  // #### General ####
  /// Line data. Give the parameters of the whole line, not of this half.
  void setParameters(Matrix resistance, Matrix inductance, Matrix capacitance);
  /// Where the far-end quantities come from: the other half's sending
  /// attributes in the same process, or an Interface in a co-simulation.
  void setCouplingSource(Attribute<Matrix>::Ptr receivingVolt,
                         Attribute<Matrix>::Ptr receivingCur);
  /// Creates the terminating resistor and the history current source
  void createSubComponents() override;
  /// Seeds the interface quantities and the sending attributes
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  /// Updates the history current from the exchanged quantities
  void step(Real time, Int timeStepCount);
  /// Records this end's quantities and publishes them one travel time delayed
  void postStep();

  // #### MNA section ####
  /// Sizes the ring buffers and seeds them from the steady state
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  /// Updates internal current variable of the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates internal voltage variable of the component
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
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
