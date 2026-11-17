/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph3_Transformer.h>
#include <dpsim-models/DP/DP_Ph3_MultiPortVTypeSSNComp.h>

namespace CPS::DP::Ph3::SSN {

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

  SimPowerComp<Complex>::Ptr clone(String copySuffix) override;

  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Matrix resistance, Matrix inductance);
  void setMagnetizingBranch(Real noLoadCurrent, Real noLoadLoss);
  void setReferenceWinding(WindingReference referenceWinding);

  void initializeFromNodesAndTerminals(Real frequency) override;

private:
  using PhaseVector = Eigen::Matrix<Complex, 3, 1>;

  Matrix mSeriesResistanceHalf;
  Matrix mSeriesInductanceHalf;
  MatrixComp mInductorAdmittance;
  MatrixComp mSeriesSystemInverse;
  MatrixComp mSeriesAdmittance;
  MatrixComp mMagnetizingInductorAdmittance;
  MatrixComp mMagnetizingConductance;
  MatrixComp mMidpointInverse;
  MatrixComp mPortAdmittance00;
  MatrixComp mPortAdmittance01;
  MatrixComp mPortAdmittance10;
  MatrixComp mPortAdmittance11;
  mutable MatrixComp mHistory0;
  mutable MatrixComp mHistory1;
  mutable MatrixComp mHistoryMagnetizing;
  mutable MatrixComp mMidpointHistory;
  mutable MatrixComp mSource0;
  mutable MatrixComp mSource1;
  Matrix mMagnetizingResistance;
  Matrix mMagnetizingInductance;
  Complex mPreviousCurrentFactor = Complex(1., 0.);
  Real mNoLoadCurrent = 0.01;
  Real mNoLoadLoss = 1e-3;
  Bool mWithResistiveLosses = false;
  UInt mReferenceTerminal = 0;
  Real mOrientationSign = 1.;
  Complex mRatioFromReference = Complex(1., 0.);
  WindingReference mReferenceWinding = WindingReference::Auto;

  void resolveWindingRoles();

protected:
  void recomputeDiscreteModel(Real omega) override;
  MatrixComp calculateHistoryVector() const override;
  void updateHistoryVector(MatrixComp &history) override;
  void updateState(const MatrixComp &uOld, const MatrixComp &uNew) override;
};

} // namespace CPS::DP::Ph3::SSN
