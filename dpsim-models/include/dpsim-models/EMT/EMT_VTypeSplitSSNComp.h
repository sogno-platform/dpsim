// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for partitioned EMT V-type SSN components.
///
/// The inherited SSN model represents a fixed LTI electrical plant. A
/// separately linearized controller drives an additional plant input through
/// a one-step output delay. Controller measurements are linear combinations of
/// the plant state and terminal voltage. The MNA Norton conductance therefore
/// remains constant even if the controller matrices change at every step.
class VTypeSplitSSNComp : public VTypeSSNComp {
private:
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

  VTypeSplitSSNComp(String uid, String name, Int inputSize, Int outputSize,
                    Int controllerStateSize, Int controllerInputSize,
                    Int controllerOutputSize,
                    Logger::Level logLevel = Logger::Level::off);

  /// Configure the fixed plant model, delayed controller input and
  /// measurement maps m = Hx*xPlant + Hu*uTerminal.
  void setSplitParameters(const Matrix &plantA, const Matrix &plantB,
                          const Matrix &plantC, const Matrix &plantD,
                          const Matrix &controllerOutputB,
                          const Matrix &measurementState,
                          const Matrix &measurementTerminal);

  Matrix buildControllerMeasurement(const Matrix &plantState,
                                    const Matrix &terminalVoltage) const;

  /// Build the affine controller model at the supplied local operating point.
  virtual void buildControllerStateSpaceModel(const Matrix &controllerState,
                                              const Matrix &measurement,
                                              Matrix &A, Matrix &B, Matrix &C,
                                              Matrix &D, Matrix &E,
                                              Matrix &F) const = 0;

  /// Evaluate the nonlinear controller output at the supplied local operating
  /// point.
  virtual void evaluateControllerOutput(const Matrix &controllerState,
                                        const Matrix &measurement,
                                        Matrix &output) const = 0;

  Matrix calculateHistoryVector() const override final;
  void updateState(const Matrix &uOld, const Matrix &uNew) override final;
  void recomputeDiscreteModel() override final;

public:
  /// Number of states in the augmented controller/plant/delay model.
  UInt getSplitStateCount() const;

  /// Augmented local transition matrix for the completed simulation step.
  const Matrix &getSplitDiscreteA() const;

  /// Augmented local input matrix for the terminal-voltage input.
  const Matrix &getSplitDiscreteB() const;

  /// Map from augmented states to the Norton history current.
  const Matrix &getSplitHistoryC() const;

  const Matrix &getControllerDiscreteA() const;
  const Matrix &getControllerDiscreteB() const;
  const Matrix &getControllerDiscreteBUsed() const;
  const Matrix &getDiscreteControllerOutputB() const;

  /// Whether the local matrices exposed for extraction change at each step.
  virtual Bool requiresStateSpaceMatrixUpdate() const { return true; }

  /// State attribute updated by the component post-step task.
  Attribute<Matrix>::Ptr getSplitStateAttribute() const;

  /// Local names in augmented controller/plant/delay state order.
  virtual std::vector<String> getSplitLocalStateNames() const = 0;

  /// Local abc blocks in augmented controller/plant/delay state order.
  virtual std::vector<SSNComp::LocalAbcStateBlock>
  getSplitLocalAbcStateBlocks() const = 0;
};

} // namespace EMT
} // namespace CPS
