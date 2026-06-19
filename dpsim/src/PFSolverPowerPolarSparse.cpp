// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim/PFSolverPowerPolarSparse.h>

#if defined(WITH_KLU)
#include <dpsim/KLUAdapter.h>
#elif defined(WITH_SPARSE)
#include <dpsim/SparseLUAdapter.h>
#else
#include <dpsim/DenseLUAdapter.h>
#endif

using namespace DPsim;
using namespace CPS;

PFSolverPowerPolarSparse::PFSolverPowerPolarSparse(
    CPS::String name, const CPS::SystemTopology &system, CPS::Real timeStep,
    CPS::Logger::Level logLevel)
    : PFSolverPowerPolar(name, system, timeStep, logLevel) {}

bool PFSolverPowerPolarSparse::isConnected(UInt k, UInt j) {
  return G(k, j) != 0.0 || B(k, j) != 0.0;
}

void PFSolverPowerPolarSparse::buildJacobianPattern() {
  UInt npqpv = mNumPQBuses + mNumPVBuses;
  std::vector<Eigen::Triplet<Real>> triplets;

  // Pattern mirrors the bus admittance sparsity; off-diagonal entries exist only
  // between connected buses. Blocks: J1 dTheta/dTheta, J2 dTheta/dV, J3 dQ/dTheta,
  // J4 dQ/dV (V increments only for PQ buses).
  for (UInt a = 0; a < npqpv; ++a) {
    UInt k = mPQPVBusIndices[a];
    triplets.emplace_back(a, a, 0.0); // J1 diagonal
    if (a < mNumPQBuses)
      triplets.emplace_back(a, a + npqpv, 0.0); // J2 diagonal
    for (UInt b = 0; b < npqpv; ++b) {          // J1 off-diagonal
      if (b != a && isConnected(k, mPQPVBusIndices[b]))
        triplets.emplace_back(a, b, 0.0);
    }
    for (UInt b = 0; b < mNumPQBuses; ++b) { // J2 off-diagonal
      if (b != a && isConnected(k, mPQPVBusIndices[b]))
        triplets.emplace_back(a, b + npqpv, 0.0);
    }
  }
  for (UInt a = 0; a < mNumPQBuses; ++a) {
    UInt k = mPQPVBusIndices[a];
    triplets.emplace_back(a + npqpv, a, 0.0);         // J3 diagonal
    triplets.emplace_back(a + npqpv, a + npqpv, 0.0); // J4 diagonal
    for (UInt b = 0; b < npqpv; ++b) {                // J3 off-diagonal
      if (b != a && isConnected(k, mPQPVBusIndices[b]))
        triplets.emplace_back(a + npqpv, b, 0.0);
    }
    for (UInt b = 0; b < mNumPQBuses; ++b) { // J4 off-diagonal
      if (b != a && isConnected(k, mPQPVBusIndices[b]))
        triplets.emplace_back(a + npqpv, b + npqpv, 0.0);
    }
  }

  mJsparse.resize(mNumUnknowns, mNumUnknowns);
  mJsparse.setFromTriplets(triplets.begin(), triplets.end());
  mJsparse.makeCompressed();
}

void PFSolverPowerPolarSparse::setUpJacobianStorage() {
  if (mNumUnknowns == 0)
    return;

  buildJacobianPattern();

#if defined(WITH_KLU)
  mLinearSolver = std::make_shared<KLUAdapter>(mSLog);
#elif defined(WITH_SPARSE)
  mLinearSolver = std::make_shared<SparseLUAdapter>(mSLog);
#else
  mLinearSolver = std::make_shared<DenseLUAdapter>(mSLog);
#endif

  // Analyze the fixed pattern once; values are refactorized each iteration.
  mLinearSolver->preprocessing(mJsparse, mVariableSystemMatrixEntries);
}

void PFSolverPowerPolarSparse::calculateJacobian() {
  UInt npqpv = mNumPQBuses + mNumPVBuses;
  double val;
  UInt k, j;

  // Pattern is fixed; clear values and refill the existing entries in place.
  mJsparse.coeffs().setZero();

  // J1: dP/dTheta
  for (UInt a = 0; a < npqpv; ++a) {
    k = mPQPVBusIndices[a];
    mJsparse.coeffRef(a, a) = -Q(k) - B(k, k) * sol_V.coeff(k) * sol_V.coeff(k);
    for (UInt b = 0; b < npqpv; ++b) {
      if (b == a)
        continue;
      j = mPQPVBusIndices[b];
      if (!isConnected(k, j))
        continue;
      val = sol_V.coeff(k) * sol_V.coeff(j) *
            (G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)) -
             B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
      mJsparse.coeffRef(a, b) = val;
    }
  }

  // J2: dP/dV
  for (UInt a = 0; a < npqpv; ++a) {
    k = mPQPVBusIndices[a];
    if (a < mNumPQBuses)
      mJsparse.coeffRef(a, a + npqpv) =
          P(k) + G(k, k) * sol_V.coeff(k) * sol_V.coeff(k);
    for (UInt b = 0; b < mNumPQBuses; ++b) {
      if (b == a)
        continue;
      j = mPQPVBusIndices[b];
      if (!isConnected(k, j))
        continue;
      val = sol_V.coeff(k) * sol_V.coeff(j) *
            (G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)) +
             B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
      mJsparse.coeffRef(a, b + npqpv) = val;
    }
  }

  // J3: dQ/dTheta
  for (UInt a = 0; a < mNumPQBuses; ++a) {
    k = mPQPVBusIndices[a];
    mJsparse.coeffRef(a + npqpv, a) =
        P(k) - G(k, k) * sol_V.coeff(k) * sol_V.coeff(k);
    for (UInt b = 0; b < npqpv; ++b) {
      if (b == a)
        continue;
      j = mPQPVBusIndices[b];
      if (!isConnected(k, j))
        continue;
      val = sol_V.coeff(k) * sol_V.coeff(j) *
            (G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)) +
             B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
      mJsparse.coeffRef(a + npqpv, b) = -val;
    }
  }

  // J4: dQ/dV
  for (UInt a = 0; a < mNumPQBuses; ++a) {
    k = mPQPVBusIndices[a];
    mJsparse.coeffRef(a + npqpv, a + npqpv) =
        Q(k) - B(k, k) * sol_V.coeff(k) * sol_V.coeff(k);
    for (UInt b = 0; b < mNumPQBuses; ++b) {
      if (b == a)
        continue;
      j = mPQPVBusIndices[b];
      if (!isConnected(k, j))
        continue;
      val = sol_V.coeff(k) * sol_V.coeff(j) *
            (G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)) -
             B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
      mJsparse.coeffRef(a + npqpv, b + npqpv) = val;
    }
  }
}

void PFSolverPowerPolarSparse::solveJacobianSystem() {
  // Full numeric factorization with pivoting on the first iteration of each run;
  // refactorize (reuse ordering + symbolic) afterwards. mIterations is 0 only on
  // the first iteration (set after the solve in PFSolver::solvePowerflow).
  if (mIterations == 0)
    mLinearSolver->factorize(mJsparse);
  else
    mLinearSolver->refactorize(mJsparse);

  Matrix rhs = mF;
  mX = mLinearSolver->solve(rhs);
}
