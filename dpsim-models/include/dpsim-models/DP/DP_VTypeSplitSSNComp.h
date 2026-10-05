// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
namespace DP {

/// Abstract base for partitioned DP V-type SSN components.
///
/// The fixed electrical plant is represented in packed-real phase-envelope
/// coordinates. A separately linearized real controller drives an additional
/// plant input through a one-step delay. Consequently, controller
/// re-linearization does not change the Norton matrix.
class VTypeSplitSSNComp : public MNASimPowerComp<Complex> {
private:
  const Int mPackedInputSize;
  const Int mPackedOutputSize;
  const Int mControllerStateSize;
  const Int mControllerInputSize;
  const Int mControllerOutputSize;

  Matrix mMeasurementState;
  Matrix mMeasurementTerminal;
  Matrix mSplitDiscreteA;
  Matrix mSplitDiscreteB;
  Matrix mSplitHistoryC;
  Matrix mControllerBdUsed;

  void recomputeControllerDiscreteModel();
  void rebuildSplitDiscreteModel(const Matrix &controllerBdUsed);

protected:
  Real mTimeStep;

  Matrix mA;
  Matrix mB;
  Matrix mC;
  Matrix mD;
  Matrix mdA;
  Matrix mdB;

  Matrix mW;
  MatrixComp mYHist;
  const Attribute<Matrix>::Ptr mX;

  Matrix mBControllerOutput;
  Matrix mdBControllerOutput;

  Matrix mControllerState;
  Matrix mControllerMeasurementOld;
  Matrix mControllerA;
  Matrix mControllerB;
  Matrix mControllerC;
  Matrix mControllerD;
  Matrix mControllerE;
  Matrix mControllerF;
  Matrix mControllerAd;
  Matrix mControllerBd;
  Matrix mControllerEd;
  Matrix mControllerOutput;
  Matrix mDelayedControllerOutput;

  VTypeSplitSSNComp(String uid, String name, Int packedInputSize,
                    Int packedOutputSize, Int controllerStateSize,
                    Int controllerInputSize, Int controllerOutputSize,
                    Logger::Level logLevel = Logger::Level::off);

  static Matrix packComplex(const MatrixComp &values);
  static MatrixComp unpackComplex(const Matrix &values);

  void setSplitParameters(const Matrix &plantA, const Matrix &plantB,
                          const Matrix &plantC, const Matrix &plantD,
                          const Matrix &controllerOutputB,
                          const Matrix &measurementState,
                          const Matrix &measurementTerminal);

  Matrix buildControllerMeasurement(const Matrix &plantState,
                                    const Matrix &terminalVoltage) const;

  virtual void buildControllerStateSpaceModel(const Matrix &controllerState,
                                              const Matrix &measurement,
                                              Matrix &A, Matrix &B, Matrix &C,
                                              Matrix &D, Matrix &E,
                                              Matrix &F) const = 0;

  virtual void evaluateControllerOutput(const Matrix &controllerState,
                                        const Matrix &measurement,
                                        Matrix &output) const = 0;

  virtual MatrixComp buildInitialInputFromNodes(Real frequency) = 0;
  virtual void updateLogAttributes(const Matrix &terminalVoltage) const;

  void recomputeDiscreteModel();

public:
  UInt getSplitStateCount() const;
  const Matrix &getSplitDiscreteA() const;
  const Matrix &getSplitDiscreteB() const;
  const Matrix &getSplitHistoryC() const;
  const Matrix &getControllerDiscreteA() const;
  const Matrix &getControllerDiscreteB() const;
  const Matrix &getControllerDiscreteBUsed() const;
  const Matrix &getDiscreteControllerOutputB() const;
  virtual Bool requiresStateSpaceMatrixUpdate() const { return true; }
  Attribute<Matrix>::Ptr getSplitStateAttribute() const;

  virtual std::vector<String> getSplitLocalStateNames() const = 0;

  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override final;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override final;
  void mnaCompPreStep(Real time, Int timeStepCount) override final;
  void mnaCompAddPostStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes,
      Attribute<Matrix>::Ptr &leftVector) override final;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override final;
};

} // namespace DP
} // namespace CPS
