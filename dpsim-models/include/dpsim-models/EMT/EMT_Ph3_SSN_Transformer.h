/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph3_Transformer.h>
#include <dpsim-models/EMT/EMT_Ph3_MultiPortVTypeSSNComp.h>

namespace CPS::EMT::Ph3::SSN {

/// Three-phase transformer represented by a single Norton multiport stamp.
/// The equivalent preserves the conventional transformer's T network and
/// magnetizing branch while eliminating its internal virtual nodes.
class Transformer final : public MultiPortVTypeSSNComp,
                          public Base::Ph3::Transformer,
                          public SharedFactory<Transformer> {
public:
  using SharedFactory<Transformer>::make;

  Transformer(String uid, String name,
              Logger::Level logLevel = Logger::Level::off,
              Bool withResistiveLosses = false);
  Transformer(String name, Logger::Level logLevel = Logger::Level::off)
      : Transformer(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String copySuffix) override;

  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Matrix resistance, Matrix inductance);
  void setMagnetizingBranch(Real noLoadCurrent, Real noLoadLoss);
  void setReferenceWinding(WindingReference referenceWinding);

  void initializeFromNodesAndTerminals(Real frequency) override;

private:
  Matrix mSeriesResistanceHalf;
  Matrix mSeriesInductanceHalf;
  Matrix mMagnetizingResistance;
  Matrix mMagnetizingInductance;
  Complex mRatioFromReference = Complex(1., 0.);
  Real mNoLoadCurrent = 0.01;
  Real mNoLoadLoss = 1e-3;
  Bool mWithResistiveLosses = false;
  UInt mReferenceTerminal = 0;
  WindingReference mReferenceWinding = WindingReference::Auto;

  void resolveWindingRoles();
};

} // namespace CPS::EMT::Ph3::SSN
