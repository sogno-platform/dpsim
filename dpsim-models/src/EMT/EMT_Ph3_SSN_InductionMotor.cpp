// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <dpsim-models/EMT/EMT_Ph3_SSN_InductionMotor.h>

using namespace CPS;

EMT::Ph3::SSN_InductionMotor::SSN_InductionMotor(String uid, String name,
                                                 Logger::Level logLevel)
    : TwoTerminalVTypeVariableSSNComp(uid, name, logLevel), mPolePairs(0),
      mNominalFrequency(0.0), mNominalMechanicalSpeed(0.0),
      mStatorResistance(0.0), mRotorResistance(0.0), mLs(0.0), mLr_dash(0.0),
      mLm(0.0), mRotorInertia(0.0), mMechanicalDamping(0.0),
      mMechanicalTorque(0.0), mInitialElectricalAngle(0.0),
      mAutoInitializeMechanicalTorque(true),
      mInductanceMatrix(
          Matrix::Zero(mElectricalStateSize, mElectricalStateSize)),
      mInverseInductanceMatrix(
          Matrix::Zero(mElectricalStateSize, mElectricalStateSize)),
      mResistanceMatrix(
          Matrix::Zero(mElectricalStateSize, mElectricalStateSize)),
      mJacobianRelativeStep(1e-6), mJacobianAbsoluteStep(1e-8),
      mElectricalPower(mAttributes->create<Real>("electrical_power")),
      mElectricalTorque(mAttributes->create<Real>("electrical_torque")),
      mMechanicalSpeedLog(mAttributes->create<Real>("mechanical_speed")),
      mElectricalAngleLog(mAttributes->create<Real>("electrical_angle")),
      mStatorCurrentD(mAttributes->create<Real>("stator_current_d")),
      mStatorCurrentQ(mAttributes->create<Real>("stator_current_q")),
      mStatorVoltageD(mAttributes->create<Real>("stator_voltage_d")),
      mStatorVoltageQ(mAttributes->create<Real>("stator_voltage_q")) {

  **mIntfVoltage = Matrix::Zero(mInputSize, 1);
  **mIntfCurrent = Matrix::Zero(mOutputSize, 1);

  **mElectricalPower = 0.0;
  **mElectricalTorque = 0.0;
  **mMechanicalSpeedLog = 0.0;
  **mElectricalAngleLog = 0.0;
  **mStatorCurrentD = 0.0;
  **mStatorCurrentQ = 0.0;
  **mStatorVoltageD = 0.0;
  **mStatorVoltageQ = 0.0;
}

std::vector<String> EMT::Ph3::SSN_InductionMotor::getLocalStateNames() const {
  return {"psi_sd", "psi_sq",           "psi_dr",
          "psi_qr", "mechanical_speed", "electrical_angle"};
}

void EMT::Ph3::SSN_InductionMotor::setParameters(
    Real nominalFrequency, Int polePairs, Real statorResistance,
    Real rotorResistance, Real statorInductance, Real rotorInductance,
    Real mutualInductance, Real rotorInertia, Real mechanicalDamping,
    Real mechanicalTorque, Real initialElectricalAngle,
    Bool autoInitializeMechanicalTorque) {

  if (nominalFrequency <= 0.0)
    throw std::invalid_argument("Nominal frequency must be positive.");
  if (polePairs <= 0)
    throw std::invalid_argument("The number of pole pairs must be positive.");

  if (statorResistance < 0.0)
    throw std::invalid_argument("Stator resistance is invalid.");

  if (statorInductance <= 0.0 || rotorInductance <= 0.0 ||
      mutualInductance <= 0.0)
    throw std::invalid_argument("Motor winding inductances must be positive.");

  if (statorInductance <= mutualInductance ||
      rotorInductance <= mutualInductance)
    throw std::invalid_argument(
        "Each total winding inductance must exceed its mutual inductance.");

  if (rotorInertia <= 0.0)
    throw std::invalid_argument("Rotor inertia must be positive.");
  if (mechanicalDamping < 0.0)
    throw std::invalid_argument("Mechanical damping must be non-negative.");

  mNominalFrequency = nominalFrequency;
  mPolePairs = polePairs;
  mNominalMechanicalSpeed =
      2.0 * PI * mNominalFrequency / static_cast<Real>(mPolePairs);

  mLm = mutualInductance;
  mLs = statorInductance;
  mLr_dash = rotorInductance;

  mRotorResistance = rotorResistance;
  mStatorResistance = statorResistance;

  mRotorInertia = rotorInertia;
  mMechanicalDamping = mechanicalDamping;
  mMechanicalTorque = mechanicalTorque;
  mInitialElectricalAngle = initialElectricalAngle;
  mAutoInitializeMechanicalTorque = autoInitializeMechanicalTorque;

  rebuildMachineMatrices();

  // Establish dimensions here. The actual local affine model is built after
  // initialization at a physical operating point.
  VTypeVariableSSNComp::setParameters(Matrix::Zero(mStateSize, mStateSize),
                                      Matrix::Zero(mStateSize, mInputSize),
                                      Matrix::Zero(mOutputSize, mStateSize),
                                      Matrix::Zero(mOutputSize, mInputSize),
                                      Matrix::Zero(mStateSize, 1),
                                      Matrix::Zero(mOutputSize, 1));
}

void EMT::Ph3::SSN_InductionMotor::rebuildMachineMatrices() {
  mInductanceMatrix.setZero();

  Real denom = mLr_dash * mLs - mLm * mLm;
  if (denom < DOUBLE_EPSILON)
    throw std::invalid_argument(
        "The induction motor inductance matrix is singular.");

  // State order: [sd, sq, rd', kq'].
  mInverseInductanceMatrix << mLr_dash, 0.0, -mLm, 0.0, 0.0, mLr_dash, 0.0,
      -mLm, -mLm, 0.0, mLs, 0.0, 0.0, -mLm, 0.0, mLs;

  mInverseInductanceMatrix *= 1. / denom;

  Eigen::FullPivLU<Matrix> decomposition(mInverseInductanceMatrix);
  if (!decomposition.isInvertible())
    throw std::invalid_argument(
        "The induction motor inverse inductance matrix is singular.");

  mInductanceMatrix = decomposition.inverse();

  Eigen::SelfAdjointEigenSolver<Matrix> eigenSolver(mInductanceMatrix);
  if (eigenSolver.info() != Eigen::Success ||
      eigenSolver.eigenvalues().minCoeff() <= 0.0)
    throw std::invalid_argument(
        "The induction motor inductance matrix is not positive definite.");

  mResistanceMatrix.setZero();
  mResistanceMatrix.diagonal() << mStatorResistance, mStatorResistance,
      mRotorResistance, mRotorResistance;
}

void EMT::Ph3::SSN_InductionMotor::setMechanicalTorque(Real mechanicalTorque) {
  mMechanicalTorque = mechanicalTorque;
  mAutoInitializeMechanicalTorque = false;
}

void EMT::Ph3::SSN_InductionMotor::setNumericalLinearizationParameters(
    Real relativeStep, Real absoluteStep) {
  if (relativeStep <= 0.0)
    throw std::invalid_argument(
        "Relative finite-difference step must be positive.");
  if (absoluteStep <= 0.0)
    throw std::invalid_argument(
        "Absolute finite-difference step must be positive.");

  mJacobianRelativeStep = relativeStep;
  mJacobianAbsoluteStep = absoluteStep;
}

Matrix EMT::Ph3::SSN_InductionMotor::getParkTransformMatrix(
    Real electricalAngle) const {
  electricalAngle = std::remainder(electricalAngle, 2.0 * PI);

  Matrix transform(2, 3);
  const Real scale = std::sqrt(2.0 / 3.0);

  transform.row(0) << scale * std::cos(electricalAngle),
      scale * std::cos(electricalAngle - 2.0 * PI / 3.0),
      scale * std::cos(electricalAngle + 2.0 * PI / 3.0);

  transform.row(1) << -scale * std::sin(electricalAngle),
      -scale * std::sin(electricalAngle - 2.0 * PI / 3.0),
      -scale * std::sin(electricalAngle + 2.0 * PI / 3.0);

  return transform;
}

Matrix EMT::Ph3::SSN_InductionMotor::getInverseParkTransformMatrix(
    Real electricalAngle) const {
  // For balanced abc quantities, the inverse of the orthonormal two-axis
  // transform is its transpose.
  return getParkTransformMatrix(electricalAngle).transpose();
}

Matrix
EMT::Ph3::SSN_InductionMotor::buildSpeedMatrix(Real electricalSpeed) const {
  Matrix speedMatrix = Matrix::Zero(mElectricalStateSize, mElectricalStateSize);

  speedMatrix(2, 3) = -electricalSpeed;
  speedMatrix(3, 2) = electricalSpeed;

  return speedMatrix;
}

void EMT::Ph3::SSN_InductionMotor::evaluateStateDerivative(
    const Matrix &x, const Matrix &u, Matrix &stateDerivative) const {
  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "Induction motor state vector has an invalid dimension.");
  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "Induction motor input vector has an invalid dimension.");

  stateDerivative.setZero(mStateSize, 1);

  const Matrix flux = x.block(0, 0, mElectricalStateSize, 1);
  const Real mechanicalSpeed = x(MechanicalSpeed, 0);
  const Real electricalSpeed = static_cast<Real>(mPolePairs) * mechanicalSpeed;

  const Matrix current = mInverseInductanceMatrix * flux;
  const Real statorCurrentD = current(0, 0);
  const Real statorCurrentQ = current(1, 0);

  const Matrix voltageDq = getParkTransformMatrix(0.0) * u; //squirrel-cage

  Matrix windingVoltage = Matrix::Zero(mElectricalStateSize, 1);
  windingVoltage(0, 0) = voltageDq(0, 0);
  windingVoltage(1, 0) = voltageDq(1, 0);
  windingVoltage(2, 0) = 0.0; // rotor short-circuited
  windingVoltage(3, 0) = 0.0;

  const Matrix fluxDerivative = (-mResistanceMatrix * mInverseInductanceMatrix +
                                 buildSpeedMatrix(electricalSpeed)) *
                                    flux +
                                windingVoltage;

  stateDerivative.block(0, 0, mElectricalStateSize, 1) = fluxDerivative;

  const Real electricalTorque =
      static_cast<Real>(mPolePairs) *
      (flux(PsiSd, 0) * statorCurrentQ - flux(PsiSq, 0) * statorCurrentD);

  // Current is positive entering the machine. In generator operation the
  // electromagnetic torque is therefore normally negative. The applied shaft
  // torque is positive in the direction of rotation.
  stateDerivative(MechanicalSpeed, 0) =
      (mMechanicalTorque + electricalTorque -
       mMechanicalDamping *
           (mechanicalSpeed -
            mNominalMechanicalSpeed)) / //steady state: synchronous speed as equilibrium point or slip?
      mRotorInertia;

  stateDerivative(ElectricalAngle, 0) = electricalSpeed;
}

void EMT::Ph3::SSN_InductionMotor::evaluateOutput(const Matrix &x,
                                                  const Matrix &u,
                                                  Matrix &output) const {
  (void)u;

  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "Induction motor state vector has an invalid dimension.");

  const Matrix flux = x.block(0, 0, mElectricalStateSize, 1);
  const Matrix current = mInverseInductanceMatrix * flux;

  Matrix statorCurrentDq(2, 1);
  statorCurrentDq << current(0, 0), current(1, 0);

  // Current entering the stator terminals, as required by the V-type stamp.
  output = getInverseParkTransformMatrix(0.0) * statorCurrentDq;
}

void EMT::Ph3::SSN_InductionMotor::calculateNumericalJacobians(
    const Matrix &x, const Matrix &u, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D) const {
  A.setZero(mStateSize, mStateSize);
  B.setZero(mStateSize, mInputSize);
  C.setZero(mOutputSize, mStateSize);
  D.setZero(mOutputSize, mInputSize);

  Matrix fPlus = Matrix::Zero(mStateSize, 1);
  Matrix fMinus = Matrix::Zero(mStateSize, 1);
  Matrix gPlus = Matrix::Zero(mOutputSize, 1);
  Matrix gMinus = Matrix::Zero(mOutputSize, 1);

  for (Int column = 0; column < mStateSize; ++column) {
    const Real step =
        mJacobianAbsoluteStep +
        mJacobianRelativeStep * std::max(1.0, std::abs(x(column, 0)));

    Matrix xPlus = x;
    Matrix xMinus = x;
    xPlus(column, 0) += step;
    xMinus(column, 0) -= step;

    evaluateStateDerivative(xPlus, u, fPlus);
    evaluateStateDerivative(xMinus, u, fMinus);
    evaluateOutput(xPlus, u, gPlus);
    evaluateOutput(xMinus, u, gMinus);

    A.col(column) = (fPlus - fMinus) / (2.0 * step);
    C.col(column) = (gPlus - gMinus) / (2.0 * step);
  }

  for (Int column = 0; column < mInputSize; ++column) {
    const Real step =
        mJacobianAbsoluteStep +
        mJacobianRelativeStep * std::max(1.0, std::abs(u(column, 0)));

    Matrix uPlus = u;
    Matrix uMinus = u;
    uPlus(column, 0) += step;
    uMinus(column, 0) -= step;

    evaluateStateDerivative(x, uPlus, fPlus);
    evaluateStateDerivative(x, uMinus, fMinus);
    evaluateOutput(x, uPlus, gPlus);
    evaluateOutput(x, uMinus, gMinus);

    B.col(column) = (fPlus - fMinus) / (2.0 * step);
    D.col(column) = (gPlus - gMinus) / (2.0 * step);
  }
}

void EMT::Ph3::SSN_InductionMotor::buildStateSpaceModel(
    const Matrix &x, const Matrix &u, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D, Matrix &E, Matrix &F) const {
  calculateNumericalJacobians(x, u, A, B, C, D);

  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);
  Matrix output = Matrix::Zero(mOutputSize, 1);

  evaluateStateDerivative(x, u, stateDerivative);
  evaluateOutput(x, u, output);

  E = stateDerivative - A * x - B * u;
  F = output - C * x - D * u;
}

Bool EMT::Ph3::SSN_InductionMotor::updateComponentParameters() {
  Matrix stateOffset;
  Matrix outputOffset;

  buildStateSpaceModel(**mX, **mIntfVoltage, mA, mB, mC, mD, stateOffset,
                       outputOffset);

  setStateOffset(stateOffset);
  setOutputOffset(outputOffset);

  return true;
}

void EMT::Ph3::SSN_InductionMotor::updateLogAttributes(const Matrix &u) const {
  const Matrix &x = **mX;
  const Matrix flux = x.block(0, 0, mElectricalStateSize, 1);
  const Matrix current = mInverseInductanceMatrix * flux;
  const Matrix voltageDq = getParkTransformMatrix(0.0) * u;

  Matrix statorCurrentDq(2, 1);
  statorCurrentDq << current(0, 0), current(1, 0);

  const Real electricalTorque =
      static_cast<Real>(mPolePairs) *
      (flux(PsiSd, 0) * current(1, 0) - flux(PsiSq, 0) * current(0, 0));

  const Real electricalPower = voltageDq(0, 0) * statorCurrentDq(0, 0) +
                               voltageDq(1, 0) * statorCurrentDq(1, 0);

  **mElectricalPower = electricalPower;
  **mElectricalTorque = electricalTorque;
  **mMechanicalSpeedLog = x(MechanicalSpeed, 0);
  **mElectricalAngleLog = std::remainder(x(ElectricalAngle, 0), 2.0 * PI);
  **mStatorCurrentD = current(0, 0);
  **mStatorCurrentQ = current(1, 0);
  **mStatorVoltageD = voltageDq(0, 0);
  **mStatorVoltageQ = voltageDq(1, 0);
}

void EMT::Ph3::SSN_InductionMotor::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");

  const MatrixComp initialVoltagePhasor = buildInitialInputFromNodes(frequency);
  const Matrix initialVoltageAbc = initialVoltagePhasor.real();

  Matrix x0 = Matrix::Zero(mStateSize, 1);
  x0(MechanicalSpeed, 0) = 0.0;
  x0(ElectricalAngle, 0) = mInitialElectricalAngle;

  **mX = x0;
  **mIntfVoltage = initialVoltageAbc;

  updateStateSpaceModel();
  mYHist = calculateHistoryVector();
  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;
  updateLogAttributes(**mIntfVoltage);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Cold-start initialization: standstill, zero flux.");
}

Matrix EMT::Ph3::SSN_InductionMotor::getState() const { return **mX; }

Matrix EMT::Ph3::SSN_InductionMotor::getStateDerivative() const {
  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);
  evaluateStateDerivative(**mX, **mIntfVoltage, stateDerivative);
  return stateDerivative;
}

Matrix EMT::Ph3::SSN_InductionMotor::getInterfaceVoltage() const {
  return **mIntfVoltage;
}

Matrix EMT::Ph3::SSN_InductionMotor::getInterfaceCurrent() const {
  return **mIntfCurrent;
}
