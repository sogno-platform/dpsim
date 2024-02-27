/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include "dpsim-models/Solver/PFSolverInterfaceBranch.h"
#include <dpsim-models/Base/Base_Ph1_PiLine.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/SP/SP_Ph1_Inductor.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>

namespace CPS {
namespace SP {
namespace Ph1 {

class RXLine : public CompositePowerComp<Complex>,
               public SharedFactory<RXLine>,
               public PFSolverInterfaceBranch,
               public Base::Ph1::PiLine {
protected:
  /// CHECK: Which of these really need to be member variables?
  ///Capacitance of the line in [F]
  Real mCapacitance;
  ///base current [V]
  Real mBaseCurrent;
  ///base apparent power [VA]
  Real mBaseApparentPower;
  ///base omega [1/s]
  Real mBaseOmega;
  ///base impedance [Ohm]
  Real mBaseImpedance;
  ///base admittance [S]
  Real mBaseAdmittance;
  ///base inductance [H]
  Real mBaseInductance;
  ///base capacitance [F]
  Real mBaseCapacitance;

  ///resistance in [pu]
  Real mSeriesResPerUnit;
  ///Capacitance of the line in [pu]
  Real mParallelCapPerUnit;
  ///Inductance of the line in [pu]
  Real mSeriesIndPerUnit;

  // #### Admittance matrix stamp ####
  MatrixComp mY_element;

  /// Inductance submodel
  std::shared_ptr<Inductor> mSubInductor;
  /// Resistor submodel
  std::shared_ptr<Resistor> mSubResistor;
  /// Inductor end to ground resistor to facilitate initialization
  std::shared_ptr<Resistor> mInitialResistor;

public:
  ///base voltage [V]
  const Attribute<Real>::Ptr mBaseVoltage;
  ///Inductance of the line in [H]
  /// CHECK: Why does this not use the base class' attribute mSeriesInd?
  const Attribute<Real>::Ptr mInductance;

  /// nodal active power injection
  const Attribute<Real>::Ptr mActivePowerInjection;
  /// nodal reactive power injection
  const Attribute<Real>::Ptr mReactivePowerInjection;

  // #### Power flow results ####
  /// branch Current flow [A], coef(0) has data from node 0, coef(1) from node 1.
  const Attribute<MatrixComp>::Ptr mCurrent;

  /// branch active powerflow [W], coef(0) has data from node 0, coef(1) from node 1.
  const Attribute<Matrix>::Ptr mActivePowerBranch;

  /// branch reactive powerflow [Var], coef(0) has data from node 0, coef(1) from node 1.
  const Attribute<Matrix>::Ptr mReactivePowerBranch;

  // #### constructors ####
  // power flow

  /// Defines UID, name, base voltage, component parameters and logging level
  RXLine(String uid, String name, Real baseVoltage, Real resistance,
         Real inductance, Logger::Level logLevel = Logger::Level::off);
  // MNA
  /// Defines UID, name, logging level
  RXLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  RXLine(String name, Logger::Level logLevel = Logger::Level::off)
      : RXLine(name, name, logLevel) {}

  // #### General ####
  /// Specify per-unit system by base voltage, base apparent power and omega (rms value for voltage expected)
  void setPerUnitSystem(Real baseApparentPower, Real baseOmega);
  /// Transform component parameters to the specified per-unit system
  void transformParametersToPerUnitSystem();

  // #### Powerflow section ####
  /// Stamps admittance matrix
  void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) override;

  /// updates branch current and power flow, input pu value, update with real value
  void updateBranchFlow(VectorComp &current, VectorComp &powerflow);
  /// stores nodal injection power in this line object
  void storeNodalInjection(Complex powerInjection);

  // #### Getter ####
  /// get admittance matrix
  MatrixComp Y_element();

  // #### MNA Section ####

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

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
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
