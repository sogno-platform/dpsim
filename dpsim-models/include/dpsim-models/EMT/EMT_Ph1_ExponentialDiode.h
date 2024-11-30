/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
//#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/Solver/MNANonlinearVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// EMT ExponentialDiode using the Shockley ideal diode equation
class ExponentialDiode : public MNASimPowerComp<Real>,
                         public SharedFactory<ExponentialDiode>,
                         public MNANonlinearVariableCompInterface {
protected:
  Matrix Jacobian = Matrix::Zero(1, 1);

public:
  //Reverse-bias saturation current. Default: mI_S = 0.000001
  const CPS::Attribute<Real>::Ptr mI_S;
  //Thermal voltage. Default: mV_T = 0.027
  const CPS::Attribute<Real>::Ptr mV_T;

  /// Defines UID, name, component parameters and logging level
  ExponentialDiode(String uid, String name,
                   Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  ExponentialDiode(String name, Logger::Level logLevel = Logger::Level::off)
      : ExponentialDiode(name, name, logLevel) {}

  // #### General ####
  ///
  SimPowerComp<Real>::Ptr clone(String name) override;
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  //Sets reverse-bias saturation current I_S and thermal voltage V_T.
  //If this method is not called, the diode will have the following
  //values by default: I_S = 0.000001 (A) and V_T = 0.027 (V).
  void setParameters(Real I_S, Real V_T);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftSideVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override {
  } //No right side vector stamps
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA pre and post step operations
  void mnaCompPreStep(Real time,
                      Int timeStepCount) override; //No right side vector stamps
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// add MNA pre and post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  virtual void iterationUpdate(const Matrix &leftVector) override;

  virtual bool hasParameterChanged() override { return true; }

  void calculateNonlinearFunctionResult() override;

  void updateJacobian();
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
