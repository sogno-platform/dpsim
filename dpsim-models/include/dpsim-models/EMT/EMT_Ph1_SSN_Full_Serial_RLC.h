// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph1_Capacitor.h>
#include <dpsim-models/Base/Base_Ph1_Inductor.h>
#include <dpsim-models/Base/Base_Ph1_Resistor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
namespace SSN {
/// \brief Full_Serial_RLC
///
/// This element represents an one port circuit consisting of a resistor,
/// an inductor and a capacitor connected in series. The terminals are at
/// the beginning and the end of the component chain.
///	The states are the capacitor voltage and the inductor current, the output
/// is the latter of those states (inductor current). The input is the voltage
/// across the whole circuit. States and past inputs are updated after each
/// time step and are used to calculate the current (input) voltage,
/// represented as MNA node voltages.
/// SSN theory and variable naming based on
/// C. Dufour, J. Mahseredjian and J. Belanger,
/// "A combined state-space nodal method for the simulation of power system
/// transients," 2011 IEEE Power and Energy Society General Meeting, Detroit,
/// MI, USA, 2011, pp. 1-1, doi: 10.1109/PES.2011.6038887. keywords:
/// {Mathematical model;Analytical models;Equations;Power system transients;
/// Couplings;Switching circuits}

class Full_Serial_RLC final : public MNASimPowerComp<Real>,
                              public SharedFactory<Full_Serial_RLC>,
                              public Base::Ph1::Resistor,
                              public Base::Ph1::Inductor,
                              public Base::Ph1::Capacitor {
public:
  /// Defines UID, name, component parameters and logging level
  Full_Serial_RLC(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
      : Full_Serial_RLC(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;
  void setParameters(Real resistance, Real inductance, Real capacitance);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

private:
  Matrix mState = Matrix::Zero(2, 1);
  Real mYHistory = 0;

  Real mDufourUNT = 0;

  Matrix mDufourAKHat = Matrix::Zero(2, 2);
  Matrix mDufourBKHat = Matrix::Zero(2, 1);
  Matrix mDufourBKNHat = Matrix::Zero(2, 1);
  Real mDufourWKN = 0;
  Matrix mDufourCKN = Matrix::Zero(1, 2);

  void ssnUpdateState();
};
} // namespace SSN
} // namespace Ph1
} // namespace EMT
} // namespace CPS
