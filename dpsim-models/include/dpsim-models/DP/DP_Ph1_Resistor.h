/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Resistor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Dynamic phasor resistor model
class Resistor : public MNASimPowerComp<Complex>,
                 public Base::Ph1::Resistor,
                 public MNATearInterface,
                 public DAEInterface,
                 public SharedFactory<Resistor> {
public:
  /// Defines UID, name and logging level
  Resistor(String uid, String name,
           Logger::Level loglevel = Logger::Level::off);
  /// Defines name and logging level
  Resistor(String name, Logger::Level logLevel = Logger::Level::off)
      : Resistor(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);

  // #### MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);
  void mnaCompInitializeHarm(Real omega, Real timeStep,
                             std::vector<Attribute<Matrix>::Ptr> leftVector);
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  /// Stamps system matrix considering the frequency index
  void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                         Int freqIdx);
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector);
  void mnaCompUpdateVoltageHarm(const Matrix &leftVector, Int freqIdx);
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector);
  void mnaCompUpdateCurrentHarm();
  /// MNA pre and post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector);
  /// add MNA pre and post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector);

  class MnaPostStepHarm : public Task {
  public:
    MnaPostStepHarm(Resistor &resistor,
                    std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**resistor.mName + ".MnaPostStepHarm"), mResistor(resistor),
          mLeftVectors(leftVectors) {
      for (UInt i = 0; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
      mModifiedAttributes.push_back(mResistor.attribute("v_intf"));
      mModifiedAttributes.push_back(mResistor.attribute("i_intf"));
    }
    void execute(Real time, Int timeStepCount);

  private:
    Resistor &mResistor;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };

  // #### MNA Tear Section ####
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) override;

  // #### DAE Section ####
  ///Residual Function for DAE Solver
  void daeResidual(double ttime, const double state[], const double dstate_dt[],
                   double resid[], std::vector<int> &off);
  ///Voltage Getter
  Complex daeInitialize();
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
