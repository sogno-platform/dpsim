// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <dpsim-models/EMT/EMT_Ph3_SSN_SynchronousGenerator.h>

using namespace CPS;

EMT::Ph3::SSN_SynchronousGenerator::SSN_SynchronousGenerator(
    String uid, String name, Logger::Level logLevel)
    : TwoTerminalVTypeVariableSSNComp(uid, name, logLevel), mPolePairs(0),
      mNominalFrequency(0.0), mNominalMechanicalSpeed(0.0),
      mStatorResistance(0.0), mFieldResistance(0.0), mDamperResistanceD(0.0),
      mDamperResistanceQ1(0.0), mDamperResistanceQ2(0.0), mLd(0.0), mLq(0.0),
      mLmd(0.0), mLmq(0.0), mLField(0.0), mLDamperD(0.0), mLDamperQ1(0.0),
      mLDamperQ2(0.0), mRotorInertia(0.0), mMechanicalDamping(0.0),
      mFieldVoltage(0.0), mMechanicalTorque(0.0), mInitialElectricalAngle(0.0),
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
      mStatorVoltageQ(mAttributes->create<Real>("stator_voltage_q")),
      mFieldCurrent(mAttributes->create<Real>("field_current")) {

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
  **mFieldCurrent = 0.0;
}

std::vector<String>
EMT::Ph3::SSN_SynchronousGenerator::getLocalStateNames() const {
  return {"psi_sd",           "psi_sq",          "psi_field",
          "psi_damper_d",     "psi_damper_q1",   "psi_damper_q2",
          "mechanical_speed", "electrical_angle"};
}

void EMT::Ph3::SSN_SynchronousGenerator::setParameters(
    Real nominalFrequency, Int polePairs, Real statorResistance,
    Real fieldResistance, Real damperResistanceD, Real damperResistanceQ1,
    Real damperResistanceQ2, Real ld, Real lq, Real lmd, Real lmq, Real lField,
    Real lDamperD, Real lDamperQ1, Real lDamperQ2, Real rotorInertia,
    Real mechanicalDamping, Real fieldVoltage, Real mechanicalTorque,
    Real initialElectricalAngle, Bool autoInitializeMechanicalTorque) {

  if (nominalFrequency <= 0.0)
    throw std::invalid_argument("Nominal frequency must be positive.");
  if (polePairs <= 0)
    throw std::invalid_argument("The number of pole pairs must be positive.");

  if (statorResistance < 0.0 || fieldResistance <= 0.0 ||
      damperResistanceD <= 0.0 || damperResistanceQ1 <= 0.0 ||
      damperResistanceQ2 <= 0.0)
    throw std::invalid_argument("Machine winding resistances are invalid.");

  if (ld <= 0.0 || lq <= 0.0 || lmd <= 0.0 || lmq <= 0.0 || lField <= 0.0 ||
      lDamperD <= 0.0 || lDamperQ1 <= 0.0 || lDamperQ2 <= 0.0)
    throw std::invalid_argument(
        "Machine winding inductances must be positive.");

  if (ld <= lmd || lq <= lmq || lField <= lmd || lDamperD <= lmd ||
      lDamperQ1 <= lmq || lDamperQ2 <= lmq)
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

  mStatorResistance = statorResistance;
  mFieldResistance = fieldResistance;
  mDamperResistanceD = damperResistanceD;
  mDamperResistanceQ1 = damperResistanceQ1;
  mDamperResistanceQ2 = damperResistanceQ2;

  mLd = ld;
  mLq = lq;
  mLmd = lmd;
  mLmq = lmq;
  mLField = lField;
  mLDamperD = lDamperD;
  mLDamperQ1 = lDamperQ1;
  mLDamperQ2 = lDamperQ2;

  mRotorInertia = rotorInertia;
  mMechanicalDamping = mechanicalDamping;
  mFieldVoltage = fieldVoltage;
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

void EMT::Ph3::SSN_SynchronousGenerator::rebuildMachineMatrices() {
  mInductanceMatrix.setZero();

  // State order: [sd, sq, field', kd', kq1', kq2'].
  mInductanceMatrix << mLd, 0.0, mLmd, mLmd, 0.0, 0.0, 0.0, mLq, 0.0, 0.0, mLmq,
      mLmq, mLmd, 0.0, mLField, mLmd, 0.0, 0.0, mLmd, 0.0, mLmd, mLDamperD, 0.0,
      0.0, 0.0, mLmq, 0.0, 0.0, mLDamperQ1, mLmq, 0.0, mLmq, 0.0, 0.0, mLmq,
      mLDamperQ2;

  Eigen::FullPivLU<Matrix> decomposition(mInductanceMatrix);
  if (!decomposition.isInvertible())
    throw std::invalid_argument(
        "The synchronous-machine inductance matrix is singular.");

  mInverseInductanceMatrix = decomposition.inverse();

  Eigen::SelfAdjointEigenSolver<Matrix> eigenSolver(mInductanceMatrix);
  if (eigenSolver.info() != Eigen::Success ||
      eigenSolver.eigenvalues().minCoeff() <= 0.0)
    throw std::invalid_argument(
        "The synchronous-machine inductance matrix is not positive definite.");

  mResistanceMatrix.setZero();
  mResistanceMatrix.diagonal() << mStatorResistance, mStatorResistance,
      mFieldResistance, mDamperResistanceD, mDamperResistanceQ1,
      mDamperResistanceQ2;
}

void EMT::Ph3::SSN_SynchronousGenerator::setFieldVoltage(Real fieldVoltage) {
  mFieldVoltage = fieldVoltage;
}

void EMT::Ph3::SSN_SynchronousGenerator::setMechanicalTorque(
    Real mechanicalTorque) {
  mMechanicalTorque = mechanicalTorque;
  mAutoInitializeMechanicalTorque = false;
}

void EMT::Ph3::SSN_SynchronousGenerator::setNumericalLinearizationParameters(
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

Matrix EMT::Ph3::SSN_SynchronousGenerator::getParkTransformMatrix(
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

Matrix EMT::Ph3::SSN_SynchronousGenerator::getInverseParkTransformMatrix(
    Real electricalAngle) const {
  // For balanced abc quantities, the inverse of the orthonormal two-axis
  // transform is its transpose.
  return getParkTransformMatrix(electricalAngle).transpose();
}

Matrix EMT::Ph3::SSN_SynchronousGenerator::buildSpeedMatrix(
    Real electricalSpeed) const {
  Matrix speedMatrix = Matrix::Zero(mElectricalStateSize, mElectricalStateSize);

  speedMatrix(0, 1) = electricalSpeed;
  speedMatrix(1, 0) = -electricalSpeed;

  return speedMatrix;
}

void EMT::Ph3::SSN_SynchronousGenerator::evaluateStateDerivative(
    const Matrix &x, const Matrix &u, Matrix &stateDerivative) const {
  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "Synchronous-generator state vector has an invalid dimension.");
  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "Synchronous-generator input vector has an invalid dimension.");

  stateDerivative.setZero(mStateSize, 1);

  const Matrix flux = x.block(0, 0, mElectricalStateSize, 1);
  const Real mechanicalSpeed = x(MechanicalSpeed, 0);
  const Real electricalAngle = x(ElectricalAngle, 0);
  const Real electricalSpeed = static_cast<Real>(mPolePairs) * mechanicalSpeed;

  const Matrix current = mInverseInductanceMatrix * flux;
  const Real statorCurrentD = current(0, 0);
  const Real statorCurrentQ = current(1, 0);

  const Matrix voltageDq = getParkTransformMatrix(electricalAngle) * u;

  Matrix windingVoltage = Matrix::Zero(mElectricalStateSize, 1);
  windingVoltage(0, 0) = voltageDq(0, 0);
  windingVoltage(1, 0) = voltageDq(1, 0);
  windingVoltage(2, 0) = mFieldVoltage;

  const Matrix fluxDerivative = (-mResistanceMatrix * mInverseInductanceMatrix +
                                 buildSpeedMatrix(electricalSpeed)) *
                                    flux +
                                windingVoltage;

  stateDerivative.block(0, 0, mElectricalStateSize, 1) = fluxDerivative;

  // The paper uses an orthonormal Park transform, so no 3/2 factor appears.
  const Real electricalTorque =
      static_cast<Real>(mPolePairs) *
      (flux(PsiSd, 0) * statorCurrentQ - flux(PsiSq, 0) * statorCurrentD);

  // Current is positive entering the machine. In generator operation the
  // electromagnetic torque is therefore normally negative. The applied shaft
  // torque is positive in the direction of rotation.
  stateDerivative(MechanicalSpeed, 0) =
      (mMechanicalTorque + electricalTorque -
       mMechanicalDamping * (mechanicalSpeed - mNominalMechanicalSpeed)) /
      mRotorInertia;

  stateDerivative(ElectricalAngle, 0) = electricalSpeed;
}

void EMT::Ph3::SSN_SynchronousGenerator::evaluateOutput(const Matrix &x,
                                                        const Matrix &u,
                                                        Matrix &output) const {
  (void)u;

  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "Synchronous-generator state vector has an invalid dimension.");

  const Matrix flux = x.block(0, 0, mElectricalStateSize, 1);
  const Matrix current = mInverseInductanceMatrix * flux;

  Matrix statorCurrentDq(2, 1);
  statorCurrentDq << current(0, 0), current(1, 0);

  // Current entering the stator terminals, as required by the V-type stamp.
  output =
      getInverseParkTransformMatrix(x(ElectricalAngle, 0)) * statorCurrentDq;
}

void EMT::Ph3::SSN_SynchronousGenerator::calculateNumericalJacobians(
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

void EMT::Ph3::SSN_SynchronousGenerator::buildStateSpaceModel(
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

Bool EMT::Ph3::SSN_SynchronousGenerator::updateComponentParameters() {
  Matrix stateOffset;
  Matrix outputOffset;

  buildStateSpaceModel(**mX, **mIntfVoltage, mA, mB, mC, mD, stateOffset,
                       outputOffset);

  setStateOffset(stateOffset);
  setOutputOffset(outputOffset);

  // Park transformation, speed term, and torque coupling make the local model
  // time varying, so the SSN equivalent must be rebuilt every step.
  return true;
}

void EMT::Ph3::SSN_SynchronousGenerator::updateLogAttributes(
    const Matrix &u) const {
  const Matrix &x = **mX;
  const Matrix flux = x.block(0, 0, mElectricalStateSize, 1);
  const Matrix current = mInverseInductanceMatrix * flux;
  const Matrix voltageDq = getParkTransformMatrix(x(ElectricalAngle, 0)) * u;

  Matrix statorCurrentDq(2, 1);
  statorCurrentDq << current(0, 0), current(1, 0);

  const Real electricalTorque =
      static_cast<Real>(mPolePairs) *
      (flux(PsiSd, 0) * current(1, 0) - flux(PsiSq, 0) * current(0, 0));

  // Positive electrical power is absorbed by the machine. Generator output is
  // therefore -electrical_power.
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
  **mFieldCurrent = current(2, 0);
}

void EMT::Ph3::SSN_SynchronousGenerator::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");

  if (std::abs(frequency - mNominalFrequency) >
      1e-6 * std::max(1.0, mNominalFrequency)) {
    SPDLOG_LOGGER_WARN(
        mSLog,
        "Initialization frequency {:.9g} Hz differs from configured "
        "nominal frequency {:.9g} Hz.",
        frequency, mNominalFrequency);
  }

  const MatrixComp initialVoltagePhasor = buildInitialInputFromNodes(frequency);
  const Matrix initialVoltageAbc = initialVoltagePhasor.real();

  Matrix x0 = Matrix::Zero(mStateSize, 1);

  const Real initialMechanicalSpeed =
      2.0 * PI * frequency / static_cast<Real>(mPolePairs);
  const Real initialElectricalSpeed =
      static_cast<Real>(mPolePairs) * initialMechanicalSpeed;

  x0(MechanicalSpeed, 0) = initialMechanicalSpeed;

  // Generator power is negative because current is positive entering the
  // component.
  const Complex targetPower = terminal(1)->singlePower();

  const Real targetActivePower = targetPower.real();
  const Real targetReactivePower = targetPower.imag();

  Real electricalAngle = mInitialElectricalAngle;
  Real fieldVoltage = mFieldVoltage;

  Matrix initialFlux = Matrix::Zero(mElectricalStateSize, 1);

  Matrix initialCurrent = Matrix::Zero(mElectricalStateSize, 1);

  constexpr Int maxIterations = 30;
  constexpr Real relativePowerTolerance = 1e-9;
  constexpr Real angleStep = 1e-6;
  constexpr Real fieldVoltageStep = 1e-4;

  auto calculateOperatingPoint = [&](Real angle, Real excitation, Matrix &flux,
                                     Matrix &current, Real &activePower,
                                     Real &reactivePower) {
    const Matrix voltageDq = getParkTransformMatrix(angle) * initialVoltageAbc;

    Matrix windingVoltage = Matrix::Zero(mElectricalStateSize, 1);

    windingVoltage(0, 0) = voltageDq(0, 0);
    windingVoltage(1, 0) = voltageDq(1, 0);
    windingVoltage(2, 0) = excitation;

    const Matrix electricalSystemMatrix =
        -mResistanceMatrix * mInverseInductanceMatrix +
        buildSpeedMatrix(initialElectricalSpeed);

    Eigen::FullPivLU<Matrix> decomposition(electricalSystemMatrix);

    if (!decomposition.isInvertible())
      throw std::runtime_error(
          "Synchronous-generator initialization matrix is singular.");

    flux = decomposition.solve(-windingVoltage);
    current = mInverseInductanceMatrix * flux;

    const Real vd = voltageDq(0, 0);
    const Real vq = voltageDq(1, 0);

    const Real id = current(0, 0);
    const Real iq = current(1, 0);

    // Orthonormal Park transform: no 3/2 factor.
    activePower = vd * id + vq * iq;
    reactivePower = vq * id - vd * iq;
  };

  for (Int iteration = 0; iteration < maxIterations; ++iteration) {
    Real activePower = 0.0;
    Real reactivePower = 0.0;

    calculateOperatingPoint(electricalAngle, fieldVoltage, initialFlux,
                            initialCurrent, activePower, reactivePower);

    Matrix residual(2, 1);
    residual << activePower - targetActivePower,
        reactivePower - targetReactivePower;

    const Real powerBase = std::max(1.0, std::abs(targetActivePower));

    if (residual.norm() / powerBase < relativePowerTolerance)
      break;

    Matrix fluxPerturbed;
    Matrix currentPerturbed;

    Real pAngle = 0.0;
    Real qAngle = 0.0;

    calculateOperatingPoint(electricalAngle + angleStep, fieldVoltage,
                            fluxPerturbed, currentPerturbed, pAngle, qAngle);

    Real pField = 0.0;
    Real qField = 0.0;

    calculateOperatingPoint(electricalAngle, fieldVoltage + fieldVoltageStep,
                            fluxPerturbed, currentPerturbed, pField, qField);

    Matrix jacobian(2, 2);

    jacobian(0, 0) = (pAngle - activePower) / angleStep;
    jacobian(1, 0) = (qAngle - reactivePower) / angleStep;

    jacobian(0, 1) = (pField - activePower) / fieldVoltageStep;
    jacobian(1, 1) = (qField - reactivePower) / fieldVoltageStep;

    Eigen::FullPivLU<Matrix> jacobianDecomposition(jacobian);

    if (!jacobianDecomposition.isInvertible())
      throw std::runtime_error(
          "Synchronous-generator operating-point Jacobian is singular.");

    const Matrix correction = jacobianDecomposition.solve(-residual);

    electricalAngle += correction(0, 0);
    fieldVoltage += correction(1, 0);
  }

  // Recalculate once using the converged values.
  Real initializedActivePower = 0.0;
  Real initializedReactivePower = 0.0;

  calculateOperatingPoint(electricalAngle, fieldVoltage, initialFlux,
                          initialCurrent, initializedActivePower,
                          initializedReactivePower);

  mFieldVoltage = fieldVoltage;

  x0(ElectricalAngle, 0) = electricalAngle;
  x0.block(0, 0, mElectricalStateSize, 1) = initialFlux;

  const Real initialElectricalTorque =
      static_cast<Real>(mPolePairs) *
      (initialFlux(PsiSd, 0) * initialCurrent(1, 0) -
       initialFlux(PsiSq, 0) * initialCurrent(0, 0));

  if (mAutoInitializeMechanicalTorque)
    mMechanicalTorque = -initialElectricalTorque;

  **mX = x0;
  **mIntfVoltage = initialVoltageAbc;

  updateStateSpaceModel();
  mYHist = calculateHistoryVector();
  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;

  updateLogAttributes(**mIntfVoltage);

  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);
  evaluateStateDerivative(**mX, **mIntfVoltage, stateDerivative);

  Matrix nonlinearOutput = Matrix::Zero(mOutputSize, 1);
  evaluateOutput(**mX, **mIntfVoltage, nonlinearOutput);

  const Matrix ssnOutput = mW * (**mIntfVoltage) + mYHist;

  Matrix equilibriumResidual = stateDerivative;
  // Absolute electrical angle advances continuously; exclude it from the
  // equilibrium residual.
  equilibriumResidual(ElectricalAngle, 0) = 0.0;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- SSN synchronous-generator initialization ---"
      "\nInput voltage u: {:s}"
      "\nInterface current y: {:s}"
      "\nState x: {:s}"
      "\nElectrical/mechanical equilibrium residual: {:.6e}"
      "\nElectrical angle derivative: {:.6e}"
      "\nNonlinear output: {:s}"
      "\nSSN output: {:s}"
      "\nOutput mismatch norm: {:.6e}"
      "\nW norm: {:.6e}"
      "\nHistory-vector norm: {:.6e}"
      "\nInitial electrical torque: {:.6e}"
      "\nApplied mechanical torque: {:.6e}"
      "\nInitial absorbed electrical power: {:.6e}"
      "\n--- SSN synchronous-generator initialization finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent), Logger::matrixToString(**mX),
      equilibriumResidual.norm(), stateDerivative(ElectricalAngle, 0),
      Logger::matrixToString(nonlinearOutput),
      Logger::matrixToString(ssnOutput), (nonlinearOutput - ssnOutput).norm(),
      mW.norm(), mYHist.norm(), initialElectricalTorque, mMechanicalTorque,
      **mElectricalPower);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Initial P = {:.12e}, target P = {:.12e}, error = {:.12e}",
                     initializedActivePower, targetActivePower,
                     initializedActivePower - targetActivePower);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Initial Q = {:.12e}, target Q = {:.12e}, error = {:.12e}",
                     initializedReactivePower, targetReactivePower,
                     initializedReactivePower - targetReactivePower);

  SPDLOG_LOGGER_INFO(
      mSLog, "Initial Te = {:.12e}, Tm = {:.12e}, torque residual = {:.12e}",
      initialElectricalTorque, mMechanicalTorque,
      mMechanicalTorque + initialElectricalTorque);

  SPDLOG_LOGGER_INFO(mSLog, "B norm = {:.12e}", mB.norm());
  SPDLOG_LOGGER_INFO(mSLog, "C norm = {:.12e}", mC.norm());
  SPDLOG_LOGGER_INFO(mSLog, "D norm = {:.12e}", mD.norm());
  SPDLOG_LOGGER_INFO(mSLog, "mdB norm = {:.12e}", mdB.norm());
  SPDLOG_LOGGER_INFO(mSLog, "W norm = {:.12e}", mW.norm());
}

Matrix EMT::Ph3::SSN_SynchronousGenerator::getState() const { return **mX; }

Matrix EMT::Ph3::SSN_SynchronousGenerator::getStateDerivative() const {
  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);
  evaluateStateDerivative(**mX, **mIntfVoltage, stateDerivative);
  return stateDerivative;
}

Matrix EMT::Ph3::SSN_SynchronousGenerator::getInterfaceVoltage() const {
  return **mIntfVoltage;
}

Matrix EMT::Ph3::SSN_SynchronousGenerator::getInterfaceCurrent() const {
  return **mIntfCurrent;
}
