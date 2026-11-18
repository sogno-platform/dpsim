/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_SSN_PiLine.h>

using namespace CPS;

DP::Ph3::SSN::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : MultiPortVTypeSSNComp(uid, name, logLevel),
      Base::Ph3::PiLine(mAttributes) {}

SimPowerComp<Complex>::Ptr DP::Ph3::SSN::PiLine::clone(String copySuffix) {
  auto copy = SharedFactory<PiLine>::make(copySuffix, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void DP::Ph3::SSN::PiLine::setParameters(const Matrix &seriesResistance,
                                         const Matrix &seriesInductance,
                                         const Matrix &parallelCapacitance,
                                         const Matrix &parallelConductance) {
  for (const Matrix *parameter : {&seriesResistance, &seriesInductance,
                                  &parallelCapacitance, &parallelConductance}) {
    if (parameter->rows() != 3 || parameter->cols() != 3)
      throw std::invalid_argument("PiLine parameter matrices must be 3 x 3.");
  }
  Base::Ph3::PiLine::setParameters(seriesResistance, seriesInductance,
                                   parallelCapacitance, parallelConductance);

  const Matrix inverseInductance = seriesInductance.inverse();
  const Matrix identity = Matrix::Identity(3, 3);
  Matrix a = -inverseInductance * seriesResistance;
  Matrix b = Matrix::Zero(3, 6);
  b.block(0, 0, 3, 3) = -inverseInductance;
  b.block(0, 3, 3, 3) = inverseInductance;
  Matrix c = Matrix::Zero(6, 3);
  c.block(0, 0, 3, 3) = -identity;
  c.block(3, 0, 3, 3) = identity;
  Matrix d = Matrix::Zero(6, 6);
  d.block(0, 0, 3, 3) = parallelConductance / 2.;
  d.block(3, 3, 3, 3) = parallelConductance / 2.;
  Matrix d1 = Matrix::Zero(6, 6);
  d1.block(0, 0, 3, 3) = parallelCapacitance / 2.;
  d1.block(3, 3, 3, 3) = parallelCapacitance / 2.;

  SSNComp::setParameters(a, b, c, d, d1);
}

void DP::Ph3::SSN::PiLine::initializeFromNodesAndTerminals(Real frequency) {
  if ((**mParallelCond)(0, 0) <= 0.) {
    **mParallelCond = Matrix::Zero(3, 3);
    (**mParallelCond).diagonal().setConstant(1e-6);
    setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap, **mParallelCond);
  }
  MultiPortVTypeSSNComp::initializeFromNodesAndTerminals(frequency);
}
