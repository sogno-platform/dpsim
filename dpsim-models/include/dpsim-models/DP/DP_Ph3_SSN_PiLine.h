/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph3_PiLine.h>
#include <dpsim-models/DP/DP_Ph3_MultiPortVTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {
namespace SSN {

/// Three-phase Pi line stamped directly as a multiport state-space Norton model.
///
/// The series R-L branch and both shunt G-C branches are represented by one
/// component, avoiding per-element tasks and composite right-vector reduction.
class PiLine final : public MultiPortVTypeSSNComp,
                     public Base::Ph3::PiLine,
                     public SharedFactory<PiLine> {
public:
  using SharedFactory<PiLine>::make;

  PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  PiLine(String name, Logger::Level logLevel = Logger::Level::off)
      : PiLine(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String copySuffix) override;

  void setParameters(const Matrix &seriesResistance,
                     const Matrix &seriesInductance,
                     const Matrix &parallelCapacitance = Matrix::Zero(3, 3),
                     const Matrix &parallelConductance = Matrix::Zero(3, 3));

  void initializeFromNodesAndTerminals(Real frequency) override;
};

} // namespace SSN
} // namespace Ph3
} // namespace DP
} // namespace CPS
