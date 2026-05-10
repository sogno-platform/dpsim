// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/EMT/EMT_Ph3_PiecewiseLinearInductor.h>

using namespace CPS;

EMT::Ph3::PiecewiseLinearInductor::PiecewiseLinearInductor(
    String uid, String name, Logger::Level logLevel)
    : TwoTerminalVTypeVariableSSNComp(uid, name, logLevel) {}

SimPowerComp<Real>::Ptr EMT::Ph3::PiecewiseLinearInductor::clone(String name) {
  auto copy = SharedFactory<PiecewiseLinearInductor>::make(name, mLogLevel);
  copy->setParameters(mFluxBreakpoints, mCurrentBreakpoints);
  return copy;
}

void EMT::Ph3::PiecewiseLinearInductor::setParameters(
    const std::vector<Real> &fluxBreakpoints,
    const std::vector<Real> &currentBreakpoints) {
  if (fluxBreakpoints.size() < 2)
    throw std::invalid_argument("At least two flux breakpoints are required.");

  if (fluxBreakpoints.size() != currentBreakpoints.size())
    throw std::invalid_argument(
        "Flux and current breakpoint vectors must have the same size.");

  constexpr Real originTolerance = 1e-12;

  if (std::abs(fluxBreakpoints.front()) > originTolerance ||
      std::abs(currentBreakpoints.front()) > originTolerance)
    throw std::invalid_argument(
        "Piecewise-linear characteristic must start at the origin.");

  for (size_t k = 0; k + 1 < fluxBreakpoints.size(); ++k) {
    if (fluxBreakpoints[k + 1] <= fluxBreakpoints[k])
      throw std::invalid_argument(
          "Flux breakpoints must be strictly increasing.");
    if (currentBreakpoints[k + 1] <= currentBreakpoints[k])
      throw std::invalid_argument(
          "Current breakpoints must be strictly increasing.");
  }

  mFluxBreakpoints = fluxBreakpoints;
  mCurrentBreakpoints = currentBreakpoints;

  Matrix aMatrix = Matrix::Zero(3, 3);
  Matrix bMatrix = Matrix::Identity(3, 3);
  Matrix cMatrix = Matrix::Zero(3, 3);
  Matrix dMatrix = Matrix::Zero(3, 3);

  SSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix);
  setOutputOffset(Matrix::Zero(3, 1));

  // Initialize C and f from the current operating point x = 0.
  updateComponentParameters();
}

std::pair<Real, Real>
EMT::Ph3::PiecewiseLinearInductor::slopeAndOffsetFromFlux(Real flux) const {
  const Real sign = (flux < 0.0) ? -1.0 : 1.0;
  const Real absFlux = std::abs(flux);

  size_t segment = mFluxBreakpoints.size() - 2;
  for (size_t k = 0; k + 1 < mFluxBreakpoints.size(); ++k) {
    if (absFlux <= mFluxBreakpoints[k + 1]) {
      segment = k;
      break;
    }
  }

  const Real dFlux = mFluxBreakpoints[segment + 1] - mFluxBreakpoints[segment];
  const Real dCurrent =
      mCurrentBreakpoints[segment + 1] - mCurrentBreakpoints[segment];

  const Real slope = dCurrent / dFlux;
  const Real intercept =
      mCurrentBreakpoints[segment] - slope * mFluxBreakpoints[segment];

  return std::make_pair(slope, sign * intercept);
}

Bool EMT::Ph3::PiecewiseLinearInductor::updateComponentParameters() {
  Matrix newC = Matrix::Zero(3, 3);
  Matrix newF = Matrix::Zero(3, 1);

  for (Int phase = 0; phase < 3; ++phase) {
    const Real flux = (**mX)(phase, 0);
    const auto [slope, offset] = slopeAndOffsetFromFlux(flux);

    newC(phase, phase) = slope;
    newF(phase, 0) = offset;
  }

  const Bool changed = (!mC.isApprox(newC)) || (!outputOffset().isApprox(newF));

  if (changed) {
    mC = newC;
    setOutputOffset(newF);
  }

  return changed;
}
