/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/SP/SP_Ph1_Load.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
/* \brief Ideal solid state transformer
    * Modelled as two loads at each side.
    * Demands Pref, Q1 from side 1, and generate Pref, Q2 to side 2.
    * Depends on the actual condition, values can be negative
    */
class SolidStateTransformer : public CompositePowerComp<Complex>,
                              public SharedFactory<SolidStateTransformer> {
private:
  ///
  std::shared_ptr<SP::Ph1::Load> mSubLoadSide1;
  ///
  std::shared_ptr<SP::Ph1::Load> mSubLoadSide2;

  /// Active power at secondary side [watt]
  Real mP2 = std::numeric_limits<double>::infinity();
  /// Nominal voltage of primary side [V]
  Real mNominalVoltageEnd1;
  /// Nominal voltage of secondary side [V]
  Real mNominalVoltageEnd2;
  // Per Unit values
  /// Active power at primary side [p.u.]
  Real mPref_perUnit;
  /// Active power at secondary side [p.u.]
  Real mP2_perUnit;
  /// Reactive power at primary side [p.u.]
  Real mQ1ref_perUnit;
  /// Reactive power at secondary side [p.u.]
  Real mQ2ref_perUnit;

public:
  /// Power [watt]
  const Attribute<Real>::Ptr mPref;
  /// Reactive power at primary side [var]
  const Attribute<Real>::Ptr mQ1ref;
  /// Reactive power at secondary side [var]
  const Attribute<Real>::Ptr mQ2ref;

  ///
  SolidStateTransformer(String uid, String name,
                        Logger::Level logLevel = Logger::Level::off);
  ///
  SolidStateTransformer(String name,
                        Logger::Level logLevel = Logger::Level::off)
      : SolidStateTransformer(name, name, logLevel){};
  ///
  SimPowerComp<Complex>::Ptr clone(String name);

  // #### Power Flow Section ####
  /// Initializes component
  void initializeFromNodesAndTerminals(Real frequency);
  ///
  void setParameters(Real nomV1, Real nomV2, Real Pref, Real Q1ref, Real Q2ref);
  ///
  void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
  ///
  Complex getNodalInjection(CPS::TopologicalNode::Ptr node);

  // // #### MNA Section ####
  // /// Initializes internal variables of the component
  // void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
  // /// Stamps system matrix
  // void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix);
  // /// Updates internal current variable of the component
  // void mnaCompUpdateCurrent(const Matrix& leftVector);
  // /// Updates internal voltage variable of the component
  // void mnaCompUpdateVoltage(const Matrix& leftVector);
};

} // namespace Ph1
} // namespace SP
} // namespace CPS
