// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace CPS;
using namespace DPsim;

namespace {
const Real K23 = std::sqrt(2.0 / 3.0);
const Real K32 = std::sqrt(1.5);

constexpr UInt Nc = 8;
constexpr UInt Np = 12;
constexpr UInt Ng = 6;
constexpr UInt Nd = 6;
constexpr UInt Nphysical = Np + Ng;
constexpr UInt Ntotal = Nc + Nphysical + Nd;

enum ControllerStateIndex : Int {
  Psi = 0,
  PhiPLL = 1,
  PFiltered = 2,
  QFiltered = 3,
  PhiD = 4,
  PhiQ = 5,
  GammaD = 6,
  GammaQ = 7
};

VectorComp eigenvalues(const Matrix &matrix) {
  Eigen::EigenSolver<Matrix> solver(matrix);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("Eigenvalue computation failed.");
  return solver.eigenvalues();
}

Real eigenvalueDistance(const VectorComp &a, const VectorComp &b) {
  const auto directed = [](const VectorComp &from, const VectorComp &to) {
    Real result = 0.0;
    for (Eigen::Index i = 0; i < from.rows(); ++i) {
      Real nearest = std::numeric_limits<Real>::max();
      for (Eigen::Index j = 0; j < to.rows(); ++j)
        nearest = std::min(nearest, std::abs(from(i) - to(j)));
      result = std::max(result, nearest);
    }
    return result;
  };
  return std::max(directed(a, b), directed(b, a));
}

MatrixComp balanced(const Complex &phaseA) {
  MatrixComp result(3, 1);
  result << phaseA, phaseA * SHIFT_TO_PHASE_B, phaseA * SHIFT_TO_PHASE_C;
  return result;
}

Matrix packedDq0Transform() {
  const Real k = std::sqrt(2.0 / 3.0);
  const Real k0 = 1.0 / std::sqrt(3.0);
  Matrix abcToDq0(3, 3);
  abcToDq0 << k, -0.5 * k, -0.5 * k, 0.0, std::sqrt(3.0) * 0.5 * k,
      -std::sqrt(3.0) * 0.5 * k, k0, k0, k0;
  Matrix packed = Matrix::Zero(6, 6);
  for (Int row = 0; row < 3; ++row) {
    for (Int column = 0; column < 3; ++column) {
      packed(2 * row, 2 * column) = abcToDq0(row, column);
      packed(2 * row + 1, 2 * column + 1) = abcToDq0(row, column);
    }
  }
  return packed;
}
} // namespace

class DPPh3SplitGFLExtractionExample {
public:
  DPPh3SplitGFLExtractionExample()
      : mFinalTime(1e-3), mFrequency(50.0), mOmega(2.0 * PI * mFrequency),
        mSourceVoltage(400.0 * RMS3PH_TO_PEAK1PH, 0.0), mRg(0.3), mLg(1.0e-3),
        mLf(2.0e-3), mCf(10.0e-6), mRf(0.2), mRc(0.2), mKpPLL(0.25),
        mKiPLL(0.2), mOmegaCutoff(mOmega), mPRef(10000.0), mQRef(5000.0),
        mKpPower(0.05), mKiPower(0.2), mKpCurrent(0.25), mKiCurrent(1.0) {}

  void run() const {
    const OperatingPoint op = operatingPoint();
    std::cout
        << "\n============================================================\n"
        << "DP Ph3 split-GFL state-space extraction comparison\n"
        << "============================================================\n"
        << "Topology: ideal DP source -> R/L grid -> split SSN GFL\n"
        << "Analysis frame: global dq0\n"
        << "Reference: independently assembled delayed dq0 model, "
           "trapezoidal discretization\n"
        << "Time steps: 1 us, 10 us, 100 us, 1 ms\n";
    const std::array<std::pair<Real, String>, 4> cases = {
        {{1e-6, "1us"}, {10e-6, "10us"}, {100e-6, "100us"}, {1e-3, "1ms"}}};
    for (const auto &[timeStep, label] : cases)
      runCase(op, timeStep, label);
  }

private:
  struct OperatingPoint {
    Complex source;
    Complex intermediate;
    Complex terminal;
    Complex vc;
    Complex iGrid;
  };

  struct ControllerMatrices {
    Matrix A;
    Matrix B;
    Matrix C;
    Matrix D;
  };

  OperatingPoint operatingPoint() const {
    const Complex z(mRc + mRg, mOmega * mLg);
    Complex vc = mSourceVoltage;
    Complex current(0.0, 0.0);
    for (Int iteration = 0; iteration < 100; ++iteration) {
      current = std::conj(Complex(mPRef, mQRef) / (1.5 * vc));
      const Complex next = mSourceVoltage + z * current;
      if (std::abs(next - vc) < 1e-13) {
        vc = next;
        break;
      }
      vc = next;
    }
    current = std::conj(Complex(mPRef, mQRef) / (1.5 * vc));
    return {mSourceVoltage,
            mSourceVoltage + Complex(0.0, mOmega * mLg) * current,
            vc - mRc * current, vc, current};
  }

  SystemTopology createSystem(const OperatingPoint &op,
                              const std::shared_ptr<DataLogger> &logger) const {
    auto pcc = SimNode<Complex>::make("pcc", PhaseType::ABC);
    auto middle = SimNode<Complex>::make("middle", PhaseType::ABC);
    auto grid = SimNode<Complex>::make("grid", PhaseType::ABC);
    pcc->setInitialVoltage(op.terminal / RMS3PH_TO_PEAK1PH);
    middle->setInitialVoltage(op.intermediate / RMS3PH_TO_PEAK1PH);
    grid->setInitialVoltage(op.source / RMS3PH_TO_PEAK1PH);

    auto source = DP::Ph3::VoltageSource::make("GridSource");
    source->setParameters(balanced(op.source), 0.0);
    auto resistor = DP::Ph3::Resistor::make("GridResistor");
    resistor->setParameters(Math::singlePhaseParameterToThreePhase(mRg));
    auto inductor = DP::Ph3::Inductor::make("GridInductor");
    inductor->setParameters(Math::singlePhaseParameterToThreePhase(mLg));
    auto inverter = DP::Ph3::SSN_GFL_Split::make("InverterSplit");
    inverter->setParameters(mLf, mCf, mRf, mRc, mOmega, mKpPLL, mKiPLL,
                            mOmegaCutoff, mPRef, mQRef, mKpPower, mKiPower,
                            mKpCurrent, mKiCurrent);
    source->connect({DP::SimNode::GND, grid});
    inductor->connect({grid, middle});
    resistor->connect({middle, pcc});
    inverter->connect({DP::SimNode::GND, pcc});

    logger->logAttribute("v_grid", grid->attribute("v"));
    logger->logAttribute("v_middle", middle->attribute("v"));
    logger->logAttribute("v_pcc", pcc->attribute("v"));
    logger->logAttribute("i_grid_inductor", inductor->attribute("i_intf"));
    logger->logAttribute("i_grid_resistor", resistor->attribute("i_intf"));
    logger->logAttribute("i_inverter", inverter->attribute("i_intf"));
    logger->logAttribute("vc_d", inverter->attribute("vc_d"));
    logger->logAttribute("vc_q", inverter->attribute("vc_q"));
    logger->logAttribute("i_grid_d", inverter->attribute("irc_d"));
    logger->logAttribute("i_grid_q", inverter->attribute("irc_q"));
    logger->logAttribute("p_inst", inverter->attribute("p_inst"));
    logger->logAttribute("q_inst", inverter->attribute("q_inst"));
    logger->logAttribute("omega_pll", inverter->attribute("omega_pll"));

    return SystemTopology(
        mFrequency, SystemNodeList{grid, middle, pcc},
        SystemComponentList{source, inductor, resistor, inverter});
  }

  ControllerMatrices
  buildAnalyticalControllerMatrices(const OperatingPoint &op) const;
  Matrix referenceMatrix(const OperatingPoint &op, Real timeStep) const;
  void runCase(const OperatingPoint &op, Real timeStep,
               const String &label) const;

  const Real mFinalTime;
  const Real mFrequency;
  const Real mOmega;
  const Complex mSourceVoltage;
  const Real mRg;
  const Real mLg;
  const Real mLf;
  const Real mCf;
  const Real mRf;
  const Real mRc;
  const Real mKpPLL;
  const Real mKiPLL;
  const Real mOmegaCutoff;
  const Real mPRef;
  const Real mQRef;
  const Real mKpPower;
  const Real mKiPower;
  const Real mKpCurrent;
  const Real mKiCurrent;
};

DPPh3SplitGFLExtractionExample::ControllerMatrices
DPPh3SplitGFLExtractionExample::buildAnalyticalControllerMatrices(
    const OperatingPoint &op) const {
  const Complex j(0.0, 1.0);
  const Real psi = std::arg(op.vc);
  const Complex rotation = std::exp(-j * psi);
  const Complex expJPsi = std::conj(rotation);
  const Complex vcDq = K32 * op.vc * rotation;
  const Complex iDq = K32 * op.iGrid * rotation;
  const Complex filterCurrent = j * mOmega * mCf * op.vc + op.iGrid;
  const Complex converterVoltage =
      op.vc + Complex(mRf, mOmega * mLf) * filterCurrent;
  const Complex converterVoltageDq = K32 * converterVoltage * rotation;
  const Complex power = vcDq * std::conj(iDq);

  Matrix state = Matrix::Zero(Nc, 1);
  state(Psi, 0) = psi;
  state(PhiPLL, 0) = 0.0;
  state(PFiltered, 0) = power.real();
  state(QFiltered, 0) = power.imag();
  state(PhiD, 0) = (iDq.real() + mKpPower * (power.real() - mPRef)) / mKiPower;
  state(PhiQ, 0) = (iDq.imag() - mKpPower * (power.imag() - mQRef)) / mKiPower;
  const Complex iReference(-mKpPower * state(PFiltered, 0) +
                               mKiPower * state(PhiD, 0) + mKpPower * mPRef,
                           mKpPower * state(QFiltered, 0) +
                               mKiPower * state(PhiQ, 0) - mKpPower * mQRef);
  state(GammaD, 0) = (converterVoltageDq.real() +
                      mKpCurrent * (iDq.real() - iReference.real())) /
                     mKiCurrent;
  state(GammaQ, 0) = (converterVoltageDq.imag() +
                      mKpCurrent * (iDq.imag() - iReference.imag())) /
                     mKiCurrent;

  Matrix A = Matrix::Zero(Nc, Nc);
  Matrix B = Matrix::Zero(Nc, 12);
  Matrix C = Matrix::Zero(Nd, Nc);
  Matrix D = Matrix::Zero(Nd, 12);
  A(Psi, Psi) = -mKpPLL * vcDq.real();
  A(Psi, PhiPLL) = mKiPLL;
  A(PhiPLL, Psi) = -vcDq.real();
  A(PFiltered, PFiltered) = -mOmegaCutoff;
  A(QFiltered, QFiltered) = -mOmegaCutoff;
  A(PhiD, PFiltered) = -1.0;
  A(PhiQ, QFiltered) = 1.0;
  A(GammaD, PFiltered) = -mKpPower;
  A(GammaD, PhiD) = mKiPower;
  A(GammaD, Psi) = -iDq.imag();
  A(GammaQ, QFiltered) = mKpPower;
  A(GammaQ, PhiQ) = mKiPower;
  A(GammaQ, Psi) = iDq.real();

  const std::array<Complex, 3> projection = {
      Complex(1.0, 0.0), SHIFT_TO_PHASE_C, SHIFT_TO_PHASE_B};
  for (Int phase = 0; phase < 3; ++phase) {
    const Complex gain = 0.5 * K23 * rotation * projection[phase];
    const std::array<Complex, 2> dVc = {gain, j * gain};
    const std::array<Complex, 2> dI = {gain, j * gain};
    for (Int part = 0; part < 2; ++part) {
      const Int voltageColumn = 2 * phase + part;
      const Int currentColumn = 6 + 2 * phase + part;
      B(Psi, voltageColumn) = mKpPLL * dVc[part].imag();
      B(PhiPLL, voltageColumn) = dVc[part].imag();
      const Complex powerVoltage = dVc[part] * std::conj(iDq);
      const Complex powerCurrent = vcDq * std::conj(dI[part]);
      B(PFiltered, voltageColumn) = mOmegaCutoff * powerVoltage.real();
      B(QFiltered, voltageColumn) = mOmegaCutoff * powerVoltage.imag();
      B(PFiltered, currentColumn) = mOmegaCutoff * powerCurrent.real();
      B(QFiltered, currentColumn) = mOmegaCutoff * powerCurrent.imag();
      B(GammaD, currentColumn) = -dI[part].real();
      B(GammaQ, currentColumn) = -dI[part].imag();
    }
  }

  const Complex voltageReferenceDq =
      -mKpCurrent * iDq +
      mKiCurrent * Complex(state(GammaD, 0), state(GammaQ, 0)) +
      mKpCurrent * iReference;
  const Complex dVoltageReferencePsi =
      j * K23 * expJPsi * (mKpCurrent * iDq + voltageReferenceDq);
  const std::array<Complex, Nc> outputStateDerivative = {
      dVoltageReferencePsi,
      Complex(0.0, 0.0),
      K23 * expJPsi * (-mKpCurrent * mKpPower),
      K23 * expJPsi * j * (mKpCurrent * mKpPower),
      K23 * expJPsi * (mKpCurrent * mKiPower),
      K23 * expJPsi * j * (mKpCurrent * mKiPower),
      K23 * expJPsi * mKiCurrent,
      K23 * expJPsi * j * mKiCurrent};

  for (Int outputPhase = 0; outputPhase < 3; ++outputPhase) {
    const Complex redistribution = std::conj(projection[outputPhase]);
    for (UInt controllerState = 0; controllerState < Nc; ++controllerState) {
      const Complex value =
          redistribution * outputStateDerivative[controllerState];
      C(2 * outputPhase, controllerState) = value.real();
      C(2 * outputPhase + 1, controllerState) = value.imag();
    }
    for (Int inputPhase = 0; inputPhase < 3; ++inputPhase) {
      const Complex gain = 0.5 * K23 * rotation * projection[inputPhase];
      const Complex dReal =
          redistribution * K23 * expJPsi * (-mKpCurrent * gain);
      const Complex dImag =
          redistribution * K23 * expJPsi * (-mKpCurrent * j * gain);
      D(2 * outputPhase, 6 + 2 * inputPhase) = dReal.real();
      D(2 * outputPhase + 1, 6 + 2 * inputPhase) = dReal.imag();
      D(2 * outputPhase, 7 + 2 * inputPhase) = dImag.real();
      D(2 * outputPhase + 1, 7 + 2 * inputPhase) = dImag.imag();
    }
  }

  return {A, B, C, D};
}

Matrix DPPh3SplitGFLExtractionExample::referenceMatrix(const OperatingPoint &op,
                                                       Real timeStep) const {
  const ControllerMatrices controller = buildAnalyticalControllerMatrices(op);
  Matrix controllerAd, controllerBd;
  Math::calculateStateSpaceTrapezoidalMatrices(
      controller.A, controller.B, timeStep, controllerAd, controllerBd);

  Matrix plantA = Matrix::Zero(Np, Np);
  Matrix plantB = Matrix::Zero(Np, 6);
  Matrix plantControllerB = Matrix::Zero(Np, Nd);
  for (Int phase = 0; phase < 3; ++phase) {
    const Int vcReal = 2 * phase;
    const Int vcImag = vcReal + 1;
    const Int ifReal = 6 + 2 * phase;
    const Int ifImag = ifReal + 1;
    const Int inputReal = 2 * phase;
    const Int inputImag = inputReal + 1;
    plantA(vcReal, vcReal) = plantA(vcImag, vcImag) = -1.0 / (mCf * mRc);
    plantA(vcReal, vcImag) = mOmega;
    plantA(vcImag, vcReal) = -mOmega;
    plantA(vcReal, ifReal) = plantA(vcImag, ifImag) = 1.0 / mCf;
    plantA(ifReal, vcReal) = plantA(ifImag, vcImag) = -1.0 / mLf;
    plantA(ifReal, ifReal) = plantA(ifImag, ifImag) = -mRf / mLf;
    plantA(ifReal, ifImag) = mOmega;
    plantA(ifImag, ifReal) = -mOmega;
    plantB(vcReal, inputReal) = plantB(vcImag, inputImag) = 1.0 / (mCf * mRc);
    plantControllerB(ifReal, inputReal) = plantControllerB(ifImag, inputImag) =
        1.0 / mLf;
  }

  Matrix plantAd, plantBd;
  Math::calculateStateSpaceTrapezoidalMatrices(plantA, plantB, timeStep,
                                               plantAd, plantBd);
  const Matrix plantLhs = Matrix::Identity(Np, Np) - 0.5 * timeStep * plantA;
  const Matrix plantBvd =
      plantLhs.fullPivLu().solve(0.5 * timeStep * plantControllerB);

  Matrix gridA = Matrix::Zero(Ng, Ng);
  Matrix gridB = Matrix::Zero(Ng, 6);
  for (Int phase = 0; phase < 3; ++phase) {
    const Int re = 2 * phase;
    const Int im = re + 1;
    gridA(re, re) = gridA(im, im) = -mRg / mLg;
    gridA(re, im) = mOmega;
    gridA(im, re) = -mOmega;
    gridB(re, re) = gridB(im, im) = 1.0 / mLg;
  }
  Matrix gridAd, gridBd;
  Math::calculateStateSpaceTrapezoidalMatrices(gridA, gridB, timeStep, gridAd,
                                               gridBd);

  // The dq0 controller measures capacitor voltage and injected grid current.
  // The packed phase-envelope plant is closed through u = vc - Rc*i_grid.
  Matrix terminalFromPhysical = Matrix::Zero(6, Nphysical);
  terminalFromPhysical.block(0, 0, 6, 6).setIdentity();
  terminalFromPhysical.block(0, Np, 6, 6) = -mRc * Matrix::Identity(6, 6);
  Matrix measurementFromPhysical = Matrix::Zero(12, Nphysical);
  measurementFromPhysical.block(0, 0, 6, 6).setIdentity();
  measurementFromPhysical.block(6, Np, 6, 6).setIdentity();

  Matrix terminalInput = Matrix::Zero(Nphysical, 6);
  terminalInput.block(0, 0, Np, 6) = plantBd;
  terminalInput.block(Np, 0, Ng, 6) = gridBd;
  Matrix delayInput = Matrix::Zero(Nphysical, Nd);
  delayInput.block(0, 0, Np, Nd) = 2.0 * plantBvd;
  const Matrix algebraicInverse =
      (Matrix::Identity(Nphysical, Nphysical) -
       terminalInput * terminalFromPhysical)
          .fullPivLu()
          .solve(Matrix::Identity(Nphysical, Nphysical));
  const Matrix physicalFromDelay = algebraicInverse * delayInput;

  Matrix historyAd = Matrix::Zero(Nphysical, Nphysical);
  historyAd.block(0, 0, Np, Np) = plantAd;
  historyAd.block(Np, Np, Ng, Ng) = gridAd;
  Matrix historyTerminal = Matrix::Zero(Nphysical, 6);
  historyTerminal.block(0, 0, Np, 6) =
      (plantAd + Matrix::Identity(Np, Np)) * plantBd;
  historyTerminal.block(Np, 0, Ng, 6) =
      (gridAd + Matrix::Identity(Ng, Ng)) * gridBd;
  Matrix historyDelay = Matrix::Zero(Nphysical, Nd);
  historyDelay.block(0, 0, Np, Nd) = 2.0 * plantAd * plantBvd;

  const Matrix terminalFromHistory = terminalFromPhysical * algebraicInverse;
  const Matrix terminalFromDelay = terminalFromPhysical * physicalFromDelay;
  const Matrix physicalAd = historyAd + historyTerminal * terminalFromHistory;
  const Matrix physicalDelay =
      historyDelay + historyTerminal * terminalFromDelay;
  const Matrix measurementFromHistory =
      measurementFromPhysical * algebraicInverse;
  const Matrix measurementFromDelay =
      measurementFromPhysical * physicalFromDelay;
  const Matrix controllerInputUpdate =
      controllerAd * controllerBd + controllerBd;
  const Matrix delayMeasurementGain =
      controller.C * controllerBd + controller.D;

  Matrix result = Matrix::Zero(Ntotal, Ntotal);
  const UInt physicalOffset = Nc;
  const UInt delayOffset = Nc + Nphysical;
  result.block(0, 0, Nc, Nc) = controllerAd;
  result.block(0, physicalOffset, Nc, Nphysical) =
      controllerInputUpdate * measurementFromHistory;
  result.block(0, delayOffset, Nc, Nd) =
      controllerInputUpdate * measurementFromDelay;
  result.block(physicalOffset, physicalOffset, Nphysical, Nphysical) =
      physicalAd;
  result.block(physicalOffset, delayOffset, Nphysical, Nd) = physicalDelay;
  result.block(delayOffset, 0, Nd, Nc) = controller.C;
  result.block(delayOffset, physicalOffset, Nd, Nphysical) =
      delayMeasurementGain * measurementFromHistory;
  result.block(delayOffset, delayOffset, Nd, Nd) =
      delayMeasurementGain * measurementFromDelay;
  // Express the independent reference in global dq0 envelope coordinates.
  // The DP envelopes already rotate synchronously, so this is a constant
  // power-invariant abc-to-dq0 transformation (applied to real and imaginary
  // parts separately), not the time-varying EMT Park transformation.
  Matrix transform = Matrix::Identity(Ntotal, Ntotal);
  const Matrix dq0 = packedDq0Transform();
  transform.block(Nc, Nc, 6, 6) = dq0;
  transform.block(Nc + 6, Nc + 6, 6, 6) = dq0;
  transform.block(Nc + 12, Nc + 12, 6, 6) = dq0;
  transform.block(Nc + Nphysical, Nc + Nphysical, 6, 6) = dq0;
  return transform * result * transform.transpose();
}

void DPPh3SplitGFLExtractionExample::runCase(const OperatingPoint &op,
                                             Real timeStep,
                                             const String &label) const {
  const String simulationName =
      "DP_Ph3_GFL_Split_StateSpaceExtraction_dt_" + label;
  Logger::setLogDir("logs/" + simulationName);
  auto logger = DataLogger::make(simulationName);
  Simulation simulation(simulationName, Logger::Level::warn);
  simulation.setSystem(createSystem(op, logger));
  simulation.addLogger(logger);
  simulation.setDomain(Domain::DP);
  simulation.setSolverType(Solver::Type::MNA);
  simulation.doInitFromNodesAndTerminals(true);
  simulation.doStateSpaceExtraction(true);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(mFinalTime);
  simulation.run();

  const auto &extractor = simulation.getStateSpaceExtractor();
  StateSpaceModalAnalysis modal(extractor);
  modal.update();
  const VectorComp extractedEigenvalues = modal.getDiscreteEigenvalues();
  const VectorComp analyticalEigenvalues =
      eigenvalues(referenceMatrix(op, timeStep));
  const Real error =
      eigenvalueDistance(analyticalEigenvalues, extractedEigenvalues);

  std::cout << "\nDelta t = " << label << "\n"
            << "  extraction time used: " << extractor.getLastExtractionTime()
            << " s\n"
            << "  extracted states: " << extractor.getStateCount() << "\n"
            << "  extracted vs analytical discrete eigenvalue max error: "
            << error << "\n"
            << "  simulation log: logs/" << simulationName << "/"
            << simulationName << ".csv\n";

  if (extractor.getStateCount() != Ntotal)
    throw std::runtime_error("Unexpected DP split-GFL state count.");
  if (error > 1e-7)
    throw std::runtime_error(
        "DP split-GFL extraction does not match the analytical reference.");
}

int main() {
  DPPh3SplitGFLExtractionExample example;
  example.run();
  return 0;
}
