// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <DPsim.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

using namespace DPsim;
using namespace CPS::DP;

namespace {

enum InverterStateIndex : Int {
  Psi = 0,
  PhiPLL = 1,
  PFiltered = 2,
  QFiltered = 3,
  PhiD = 4,
  PhiQ = 5,
  GammaD = 6,
  GammaQ = 7,
  VcRe = 8,
  VcIm = 9,
  IfRe = 10,
  IfIm = 11
};

struct SteadyState {
  Complex sourceVoltage;
  Complex midVoltage;
  Complex pccVoltage;
  Complex filterVoltage;
  Complex gridCurrent;
  Matrix inverterState;
};

void printEigenvalues(const String &title, const CPS::VectorComp &eigenvalues) {
  std::cout << "\n" << title << "\n";
  for (Eigen::Index idx = 0; idx < eigenvalues.rows(); ++idx)
    std::cout << "  [" << idx << "] " << eigenvalues(idx) << "\n";
}

CPS::VectorComp finiteEigenvalues(const CPS::VectorComp &eigenvalues) {
  std::vector<Complex> finiteValues;

  for (Eigen::Index idx = 0; idx < eigenvalues.rows(); ++idx) {
    const Complex value = eigenvalues(idx);
    if (std::isfinite(value.real()) && std::isfinite(value.imag()))
      finiteValues.push_back(value);
  }

  CPS::VectorComp result(finiteValues.size());
  for (Eigen::Index idx = 0; idx < result.rows(); ++idx)
    result(idx) = finiteValues[static_cast<std::size_t>(idx)];

  return result;
}

Real maxNearestDistance(const CPS::VectorComp &reference,
                        const CPS::VectorComp &actual) {
  Real maxDistance = 0.0;

  for (Eigen::Index refIdx = 0; refIdx < reference.rows(); ++refIdx) {
    Real bestDistance = std::numeric_limits<Real>::max();

    for (Eigen::Index actualIdx = 0; actualIdx < actual.rows(); ++actualIdx) {
      bestDistance = std::min(bestDistance,
                              std::abs(reference(refIdx) - actual(actualIdx)));
    }

    maxDistance = std::max(maxDistance, bestDistance);
  }

  return maxDistance;
}

Complex mapContinuousToDiscrete(const Complex &lambda, Real timeStep) {
  const Complex two(2.0, 0.0);
  return (two + timeStep * lambda) / (two - timeStep * lambda);
}

CPS::VectorComp mapContinuousToDiscrete(const CPS::VectorComp &lambda,
                                        Real timeStep) {
  CPS::VectorComp result(lambda.rows());

  for (Eigen::Index idx = 0; idx < lambda.rows(); ++idx)
    result(idx) = mapContinuousToDiscrete(lambda(idx), timeStep);

  return result;
}

CPS::VectorComp continuousEigenvalues(const Matrix &aMatrix) {
  Eigen::EigenSolver<Matrix> eigenSolver(aMatrix);
  return eigenSolver.eigenvalues();
}

} // namespace

class DPPh1InverterStateSpaceExtractionExample {
public:
  DPPh1InverterStateSpaceExtractionExample()
      : mTimeStep(100e-6), mFinalTime(1e-3), mFrequency(50.0),
        mOmega(2.0 * PI * mFrequency),
        mSourceVoltage(RMS3PH_TO_PEAK1PH * 400.0, 0.0), mGridResistance(0.3),
        mGridInductance(0.1e-3), mLf(2e-3), mCf(10e-6), mRf(0.2), mRc(0.2),
        mKpPLL(0.25), mKiPLL(0.2), mOmegaCutoff(mOmega), mPRef(10000.0),
        mQRef(5000.0), mKpPowerCtrl(0.05), mKiPowerCtrl(0.2), mKpCurrCtrl(0.25),
        mKiCurrCtrl(1.0) {}

  void run() const {
    const SteadyState steadyState = calculateSteadyState();

    const String simName = "DP_Ph1_Inverter_StateSpaceExtraction";
    Logger::setLogDir("logs/" + simName);

    auto logger = DataLogger::make(simName);

    Simulation sim(simName, Logger::Level::warn);
    sim.setSystem(createSystem(steadyState, logger));
    sim.addLogger(logger);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(mTimeStep);
    sim.setFinalTime(mFinalTime);
    sim.doStateSpaceExtraction(true);
    sim.doSystemMatrixRecomputation(true);
    sim.doInitFromNodesAndTerminals(true);
    sim.run();

    const auto &extractor = sim.getStateSpaceExtractor();
    StateSpaceModalAnalysis modalAnalysis(extractor);
    modalAnalysis.update();

    const CPS::VectorComp extractedZ = modalAnalysis.getDiscreteEigenvalues();
    const CPS::VectorComp extractedLambda =
        modalAnalysis.getContinuousEigenvalues();
    const CPS::VectorComp extractedFiniteLambda =
        finiteEigenvalues(extractedLambda);

    const Matrix analyticalNativeA = buildAnalyticalStateMatrix(steadyState);
    const Matrix analyticalDqA =
        buildAnalyticalDqStateMatrix(steadyState, analyticalNativeA);

    const CPS::VectorComp analyticalNativeLambda =
        continuousEigenvalues(analyticalNativeA);
    const CPS::VectorComp analyticalDqLambda =
        continuousEigenvalues(analyticalDqA);
    const CPS::VectorComp analyticalDqZ =
        mapContinuousToDiscrete(analyticalDqLambda, mTimeStep);

    std::cout
        << "\n============================================================\n";
    std::cout << "DP Ph1 inverter state-space extraction\n";
    std::cout
        << "============================================================\n";
    std::cout << "Topology: ideal DP voltage source -> R/L grid branch -> "
                 "DP Ph1 averaged inverter\n";
    std::cout << "Number of extraction states: " << extractor.getStateCount()
              << "\n";
    std::cout << "Simulation log written to logs/" << simName << "/" << simName
              << ".csv\n";

    std::cout << "\nOperating point:\n";
    std::cout << "  source voltage = " << steadyState.sourceVoltage << "\n";
    std::cout << "  PCC voltage    = " << steadyState.pccVoltage << "\n";
    std::cout << "  filter voltage = " << steadyState.filterVoltage << "\n";
    std::cout << "  grid current   = " << steadyState.gridCurrent << "\n";

    printEigenvalues("Extracted discrete-time eigenvalues z:", extractedZ);
    printEigenvalues("Extracted continuous-time native DP eigenvalues lambda:",
                     extractedLambda);
    printEigenvalues(
        "Analytical continuous-time native DP/mixed-frame eigenvalues lambda:",
        analyticalNativeLambda);
    printEigenvalues("Analytical continuous-time dq-frame eigenvalues lambda:",
                     analyticalDqLambda);
    printEigenvalues(
        "Analytical dq-frame trapezoidal discrete-time eigenvalues z:",
        analyticalDqZ);

    std::cout << "\nMaximum nearest-neighbour difference between extracted "
                 "finite lambda and analytical native DP/mixed-frame lambda: "
              << maxNearestDistance(analyticalNativeLambda,
                                    extractedFiniteLambda)
              << "\n";
    std::cout << "Maximum nearest-neighbour difference between extracted "
                 "finite lambda and analytical dq-frame lambda: "
              << maxNearestDistance(analyticalDqLambda, extractedFiniteLambda)
              << "\n";
    std::cout << "Maximum nearest-neighbour difference between analytical "
                 "native DP/mixed-frame lambda and analytical dq-frame lambda: "
              << maxNearestDistance(analyticalNativeLambda, analyticalDqLambda)
              << "\n";
    std::cout << "Maximum nearest-neighbour difference between extracted z "
                 "and analytical dq-frame z: "
              << maxNearestDistance(analyticalDqZ, extractedZ) << "\n";
  }

private:
  SystemTopology createSystem(const SteadyState &steadyState,
                              std::shared_ptr<DataLogger> logger) const {
    auto nGrid = SimNode::make("nGrid");
    auto nMid = SimNode::make("nMid");
    auto nPcc = SimNode::make("nPcc");

    nGrid->setInitialVoltage(steadyState.sourceVoltage);
    nMid->setInitialVoltage(steadyState.midVoltage);
    nPcc->setInitialVoltage(steadyState.pccVoltage);

    auto slack = Ph1::VoltageSource::make("Slack");
    // The DP source value is a native-frame envelope, so it must stay
    // constant in the DP simulation. Passing mFrequency here would rotate
    // the source phasor once more.
    slack->setParameters(mSourceVoltage, 0.0);

    auto gridResistance = Ph1::Resistor::make("GridResistance");
    gridResistance->setParameters(mGridResistance);

    auto gridInductance = Ph1::Inductor::make("GridInductance");
    gridInductance->setParameters(mGridInductance);

    auto inverter = Ph1::AvVoltSourceInverterStateSpace::make(
        "Inverter", Logger::Level::warn);
    inverter->setParameters(mLf, mCf, mRf, mRc, mOmega, mKpPLL, mKiPLL,
                            mOmegaCutoff, mPRef, mQRef, mKpPowerCtrl,
                            mKiPowerCtrl, mKpCurrCtrl, mKiCurrCtrl);

    slack->connect(SimNode::List{SimNode::GND, nGrid});
    gridResistance->connect(SimNode::List{nGrid, nMid});
    gridInductance->connect(SimNode::List{nMid, nPcc});
    inverter->connect(SimNode::List{SimNode::GND, nPcc});

    logger->logAttribute("v_grid", nGrid->attribute("v"));
    logger->logAttribute("v_mid", nMid->attribute("v"));
    logger->logAttribute("v_pcc", nPcc->attribute("v"));
    logger->logAttribute("i_grid_resistance",
                         gridResistance->attribute("i_intf"));
    logger->logAttribute("i_grid_inductance",
                         gridInductance->attribute("i_intf"));
    logger->logAttribute("i_inv", inverter->attribute("i_intf"));
    logger->logAttribute("vc_d", inverter->attribute("vc_d"));
    logger->logAttribute("vc_q", inverter->attribute("vc_q"));
    logger->logAttribute("irc_d", inverter->attribute("irc_d"));
    logger->logAttribute("irc_q", inverter->attribute("irc_q"));
    logger->logAttribute("p_inst", inverter->attribute("p_inst"));
    logger->logAttribute("q_inst", inverter->attribute("q_inst"));
    logger->logAttribute("omega_pll", inverter->attribute("omega_pll"));

    return SystemTopology(
        mFrequency, SystemNodeList{nGrid, nMid, nPcc},
        SystemComponentList{slack, gridResistance, gridInductance, inverter});
  }

  SteadyState calculateSteadyState() const {
    const Complex j(0.0, 1.0);
    const Complex powerRef(mPRef, mQRef);
    const Complex gridImpedance(mGridResistance, mOmega * mGridInductance);
    const Complex totalImpedance = gridImpedance + mRc;

    Complex vc = mSourceVoltage;
    Complex current(0.0, 0.0);

    for (Int iter = 0; iter < 20; ++iter) {
      current = std::conj(powerRef / vc);
      const Complex vcNext = mSourceVoltage + totalImpedance * current;

      if (std::abs(vcNext - vc) < 1e-9) {
        vc = vcNext;
        break;
      }

      vc = vcNext;
    }

    current = std::conj(powerRef / vc);

    const Complex pccVoltage = vc - mRc * current;
    const Complex midVoltage = mSourceVoltage + mGridResistance * current;
    const Complex filterCurrent = j * mOmega * mCf * vc + current;
    const Complex vRef = vc + (mRf + j * mOmega * mLf) * filterCurrent;

    const Real psi0 = std::arg(vc);
    const Complex rot0 = std::exp(-j * psi0);
    const Complex vcDQ = vc * rot0;
    const Complex currentDQ = current * rot0;
    const Complex vRefDQ = vRef * rot0;

    const Real pInit =
        vcDQ.real() * currentDQ.real() + vcDQ.imag() * currentDQ.imag();
    const Real qInit =
        -vcDQ.real() * currentDQ.imag() + vcDQ.imag() * currentDQ.real();
    const Real phiPLL0 = 0.0;
    const Real phiD0 =
        (currentDQ.real() + mKpPowerCtrl * (pInit - mPRef)) / mKiPowerCtrl;
    const Real phiQ0 =
        (currentDQ.imag() - mKpPowerCtrl * (qInit - mQRef)) / mKiPowerCtrl;
    const Real iRefD0 =
        -mKpPowerCtrl * pInit + mKiPowerCtrl * phiD0 + mKpPowerCtrl * mPRef;
    const Real iRefQ0 =
        mKpPowerCtrl * qInit + mKiPowerCtrl * phiQ0 - mKpPowerCtrl * mQRef;
    const Real gammaD0 =
        (vRefDQ.real() + mKpCurrCtrl * (currentDQ.real() - iRefD0)) /
        mKiCurrCtrl;
    const Real gammaQ0 =
        (vRefDQ.imag() + mKpCurrCtrl * (currentDQ.imag() - iRefQ0)) /
        mKiCurrCtrl;

    Matrix x0 = Matrix::Zero(12, 1);
    x0(Psi, 0) = psi0;
    x0(PhiPLL, 0) = phiPLL0;
    x0(PFiltered, 0) = pInit;
    x0(QFiltered, 0) = qInit;
    x0(PhiD, 0) = phiD0;
    x0(PhiQ, 0) = phiQ0;
    x0(GammaD, 0) = gammaD0;
    x0(GammaQ, 0) = gammaQ0;
    x0(VcRe, 0) = vc.real();
    x0(VcIm, 0) = vc.imag();
    x0(IfRe, 0) = filterCurrent.real();
    x0(IfIm, 0) = filterCurrent.imag();

    return {mSourceVoltage, midVoltage, pccVoltage, vc, current, x0};
  }

  Matrix buildAnalyticalStateMatrix(const SteadyState &steadyState) const {
    Matrix inverterA;
    Matrix inverterB;
    buildInverterJacobian(steadyState.inverterState, steadyState.pccVoltage,
                          inverterA, inverterB);

    Matrix result = Matrix::Zero(14, 14);

    Matrix duDx = Matrix::Zero(2, 12);
    duDx(0, VcRe) = 1.0;
    duDx(1, VcIm) = 1.0;

    Matrix duDi = Matrix::Zero(2, 2);
    duDi(0, 0) = -mRc;
    duDi(1, 1) = -mRc;

    result.block(0, 0, 12, 12) = inverterA + inverterB * duDx;
    result.block(0, 12, 12, 2) = inverterB * duDi;

    result(12, VcRe) = 1.0 / mGridInductance;
    result(13, VcIm) = 1.0 / mGridInductance;
    result(12, 12) = -(mGridResistance + mRc) / mGridInductance;
    result(12, 13) = mOmega;
    result(13, 12) = -mOmega;
    result(13, 13) = -(mGridResistance + mRc) / mGridInductance;

    return result;
  }

  Matrix buildAnalyticalDqStateMatrix(const SteadyState &steadyState,
                                      const Matrix &nativeA) const {
    const Matrix nativeToDq = buildNativeToDqTransformation(steadyState);
    return nativeToDq * nativeA * nativeToDq.inverse();
  }

  Matrix buildNativeToDqTransformation(const SteadyState &steadyState) const {
    Matrix transformation = Matrix::Identity(14, 14);

    const Real psi = steadyState.inverterState(Psi, 0);
    const Real cosPsi = std::cos(psi);
    const Real sinPsi = std::sin(psi);

    const Complex filterVoltage(steadyState.inverterState(VcRe, 0),
                                steadyState.inverterState(VcIm, 0));
    const Complex filterCurrent(steadyState.inverterState(IfRe, 0),
                                steadyState.inverterState(IfIm, 0));

    stampNativeToDqStateTransformation(transformation, VcRe, VcIm, VcRe, VcIm,
                                       filterVoltage, cosPsi, sinPsi);
    stampNativeToDqStateTransformation(transformation, IfRe, IfIm, IfRe, IfIm,
                                       filterCurrent, cosPsi, sinPsi);
    stampNativeToDqStateTransformation(transformation, 12, 13, 12, 13,
                                       steadyState.gridCurrent, cosPsi, sinPsi);

    return transformation;
  }

  void stampNativeToDqStateTransformation(Matrix &transformation, UInt dIndex,
                                          UInt qIndex, UInt reIndex,
                                          UInt imIndex,
                                          const Complex &steadyStateValue,
                                          Real cosPsi, Real sinPsi) const {
    const Real dValue =
        steadyStateValue.real() * cosPsi + steadyStateValue.imag() * sinPsi;
    const Real qValue =
        steadyStateValue.imag() * cosPsi - steadyStateValue.real() * sinPsi;

    transformation.row(dIndex).setZero();
    transformation.row(qIndex).setZero();

    transformation(dIndex, Psi) = qValue;
    transformation(dIndex, reIndex) = cosPsi;
    transformation(dIndex, imIndex) = sinPsi;

    transformation(qIndex, Psi) = -dValue;
    transformation(qIndex, reIndex) = -sinPsi;
    transformation(qIndex, imIndex) = cosPsi;
  }

  void buildInverterJacobian(const Matrix &x, const Complex &u, Matrix &A,
                             Matrix &B) const {
    const Real psi = x(Psi, 0);
    const Real pF = x(PFiltered, 0);
    const Real qF = x(QFiltered, 0);
    const Real phiD = x(PhiD, 0);
    const Real phiQ = x(PhiQ, 0);
    const Real gammaD = x(GammaD, 0);
    const Real gammaQ = x(GammaQ, 0);
    const Real vcRe = x(VcRe, 0);
    const Real vcIm = x(VcIm, 0);
    const Real uRe = u.real();
    const Real uIm = u.imag();
    const Real cosPsi = std::cos(psi);
    const Real sinPsi = std::sin(psi);

    const Real ircRe = (vcRe - uRe) / mRc;
    const Real ircIm = (vcIm - uIm) / mRc;
    const Real ircD = ircRe * cosPsi + ircIm * sinPsi;
    const Real ircQ = ircIm * cosPsi - ircRe * sinPsi;

    const Real dVcQByPsi = -vcIm * sinPsi - vcRe * cosPsi;
    const Real dVcQByVcRe = -sinPsi;
    const Real dVcQByVcIm = cosPsi;

    const Real dIrcDByPsi = -ircRe * sinPsi + ircIm * cosPsi;
    const Real dIrcDByVcRe = cosPsi / mRc;
    const Real dIrcDByVcIm = sinPsi / mRc;
    const Real dIrcDByURe = -cosPsi / mRc;
    const Real dIrcDByUIm = -sinPsi / mRc;
    const Real dIrcQByPsi = -ircIm * sinPsi - ircRe * cosPsi;
    const Real dIrcQByVcRe = -sinPsi / mRc;
    const Real dIrcQByVcIm = cosPsi / mRc;
    const Real dIrcQByURe = sinPsi / mRc;
    const Real dIrcQByUIm = -cosPsi / mRc;

    const Real dPByVcRe = (2.0 / mRc) * vcRe - (1.0 / mRc) * uRe;
    const Real dPByVcIm = (2.0 / mRc) * vcIm - (1.0 / mRc) * uIm;
    const Real dPByURe = -(1.0 / mRc) * vcRe;
    const Real dPByUIm = -(1.0 / mRc) * vcIm;
    const Real dQByVcRe = (1.0 / mRc) * uIm;
    const Real dQByVcIm = -(1.0 / mRc) * uRe;
    const Real dQByURe = -(1.0 / mRc) * vcIm;
    const Real dQByUIm = (1.0 / mRc) * vcRe;

    const Real iRefD =
        -mKpPowerCtrl * pF + mKiPowerCtrl * phiD + mKpPowerCtrl * mPRef;
    const Real iRefQ =
        mKpPowerCtrl * qF + mKiPowerCtrl * phiQ - mKpPowerCtrl * mQRef;
    const Real vRefD =
        -mKpCurrCtrl * ircD + mKiCurrCtrl * gammaD + mKpCurrCtrl * iRefD;
    const Real vRefQ =
        -mKpCurrCtrl * ircQ + mKiCurrCtrl * gammaQ + mKpCurrCtrl * iRefQ;

    const Real dVRefDByPsi = -mKpCurrCtrl * dIrcDByPsi;
    const Real dVRefDByVcRe = -mKpCurrCtrl * dIrcDByVcRe;
    const Real dVRefDByVcIm = -mKpCurrCtrl * dIrcDByVcIm;
    const Real dVRefDByURe = -mKpCurrCtrl * dIrcDByURe;
    const Real dVRefDByUIm = -mKpCurrCtrl * dIrcDByUIm;
    const Real dVRefDByPF = -mKpCurrCtrl * mKpPowerCtrl;
    const Real dVRefDByPhiD = mKpCurrCtrl * mKiPowerCtrl;
    const Real dVRefDByGammaD = mKiCurrCtrl;
    const Real dVRefQByPsi = -mKpCurrCtrl * dIrcQByPsi;
    const Real dVRefQByVcRe = -mKpCurrCtrl * dIrcQByVcRe;
    const Real dVRefQByVcIm = -mKpCurrCtrl * dIrcQByVcIm;
    const Real dVRefQByURe = -mKpCurrCtrl * dIrcQByURe;
    const Real dVRefQByUIm = -mKpCurrCtrl * dIrcQByUIm;
    const Real dVRefQByQF = mKpCurrCtrl * mKpPowerCtrl;
    const Real dVRefQByPhiQ = mKpCurrCtrl * mKiPowerCtrl;
    const Real dVRefQByGammaQ = mKiCurrCtrl;

    const Real dVRefEnvReByPsi = dVRefDByPsi * cosPsi - vRefD * sinPsi -
                                 dVRefQByPsi * sinPsi - vRefQ * cosPsi;
    const Real dVRefEnvImByPsi = dVRefDByPsi * sinPsi + vRefD * cosPsi +
                                 dVRefQByPsi * cosPsi - vRefQ * sinPsi;

    auto dVRefEnvRe = [&](Real dD, Real dQ) {
      return dD * cosPsi - dQ * sinPsi;
    };
    auto dVRefEnvIm = [&](Real dD, Real dQ) {
      return dD * sinPsi + dQ * cosPsi;
    };

    A = Matrix::Zero(12, 12);
    B = Matrix::Zero(12, 2);

    A(Psi, Psi) = mKpPLL * dVcQByPsi;
    A(Psi, PhiPLL) = mKiPLL;
    A(Psi, VcRe) = mKpPLL * dVcQByVcRe;
    A(Psi, VcIm) = mKpPLL * dVcQByVcIm;

    A(PhiPLL, Psi) = dVcQByPsi;
    A(PhiPLL, VcRe) = dVcQByVcRe;
    A(PhiPLL, VcIm) = dVcQByVcIm;

    A(PFiltered, PFiltered) = -mOmegaCutoff;
    A(PFiltered, VcRe) = mOmegaCutoff * dPByVcRe;
    A(PFiltered, VcIm) = mOmegaCutoff * dPByVcIm;
    B(PFiltered, 0) = mOmegaCutoff * dPByURe;
    B(PFiltered, 1) = mOmegaCutoff * dPByUIm;

    A(QFiltered, QFiltered) = -mOmegaCutoff;
    A(QFiltered, VcRe) = mOmegaCutoff * dQByVcRe;
    A(QFiltered, VcIm) = mOmegaCutoff * dQByVcIm;
    B(QFiltered, 0) = mOmegaCutoff * dQByURe;
    B(QFiltered, 1) = mOmegaCutoff * dQByUIm;

    A(PhiD, PFiltered) = -1.0;
    A(PhiQ, QFiltered) = 1.0;

    A(GammaD, PFiltered) = -mKpPowerCtrl;
    A(GammaD, PhiD) = mKiPowerCtrl;
    A(GammaD, Psi) = -dIrcDByPsi;
    A(GammaD, VcRe) = -dIrcDByVcRe;
    A(GammaD, VcIm) = -dIrcDByVcIm;
    B(GammaD, 0) = -dIrcDByURe;
    B(GammaD, 1) = -dIrcDByUIm;

    A(GammaQ, QFiltered) = mKpPowerCtrl;
    A(GammaQ, PhiQ) = mKiPowerCtrl;
    A(GammaQ, Psi) = -dIrcQByPsi;
    A(GammaQ, VcRe) = -dIrcQByVcRe;
    A(GammaQ, VcIm) = -dIrcQByVcIm;
    B(GammaQ, 0) = -dIrcQByURe;
    B(GammaQ, 1) = -dIrcQByUIm;

    A(VcRe, VcRe) = -1.0 / (mCf * mRc);
    A(VcRe, VcIm) = mOmega;
    A(VcRe, IfRe) = 1.0 / mCf;
    B(VcRe, 0) = 1.0 / (mCf * mRc);

    A(VcIm, VcRe) = -mOmega;
    A(VcIm, VcIm) = -1.0 / (mCf * mRc);
    A(VcIm, IfIm) = 1.0 / mCf;
    B(VcIm, 1) = 1.0 / (mCf * mRc);

    A(IfRe, Psi) = dVRefEnvReByPsi / mLf;
    A(IfRe, PFiltered) = dVRefEnvRe(dVRefDByPF, 0.0) / mLf;
    A(IfRe, QFiltered) = dVRefEnvRe(0.0, dVRefQByQF) / mLf;
    A(IfRe, PhiD) = dVRefEnvRe(dVRefDByPhiD, 0.0) / mLf;
    A(IfRe, PhiQ) = dVRefEnvRe(0.0, dVRefQByPhiQ) / mLf;
    A(IfRe, GammaD) = dVRefEnvRe(dVRefDByGammaD, 0.0) / mLf;
    A(IfRe, GammaQ) = dVRefEnvRe(0.0, dVRefQByGammaQ) / mLf;
    A(IfRe, VcRe) = dVRefEnvRe(dVRefDByVcRe, dVRefQByVcRe) / mLf - 1.0 / mLf;
    A(IfRe, VcIm) = dVRefEnvRe(dVRefDByVcIm, dVRefQByVcIm) / mLf;
    A(IfRe, IfRe) = -mRf / mLf;
    A(IfRe, IfIm) = mOmega;
    B(IfRe, 0) = dVRefEnvRe(dVRefDByURe, dVRefQByURe) / mLf;
    B(IfRe, 1) = dVRefEnvRe(dVRefDByUIm, dVRefQByUIm) / mLf;

    A(IfIm, Psi) = dVRefEnvImByPsi / mLf;
    A(IfIm, PFiltered) = dVRefEnvIm(dVRefDByPF, 0.0) / mLf;
    A(IfIm, QFiltered) = dVRefEnvIm(0.0, dVRefQByQF) / mLf;
    A(IfIm, PhiD) = dVRefEnvIm(dVRefDByPhiD, 0.0) / mLf;
    A(IfIm, PhiQ) = dVRefEnvIm(0.0, dVRefQByPhiQ) / mLf;
    A(IfIm, GammaD) = dVRefEnvIm(dVRefDByGammaD, 0.0) / mLf;
    A(IfIm, GammaQ) = dVRefEnvIm(0.0, dVRefQByGammaQ) / mLf;
    A(IfIm, VcRe) = dVRefEnvIm(dVRefDByVcRe, dVRefQByVcRe) / mLf;
    A(IfIm, VcIm) = dVRefEnvIm(dVRefDByVcIm, dVRefQByVcIm) / mLf - 1.0 / mLf;
    A(IfIm, IfRe) = -mOmega;
    A(IfIm, IfIm) = -mRf / mLf;
    B(IfIm, 0) = dVRefEnvIm(dVRefDByURe, dVRefQByURe) / mLf;
    B(IfIm, 1) = dVRefEnvIm(dVRefDByUIm, dVRefQByUIm) / mLf;
  }

  Real mTimeStep;
  Real mFinalTime;
  Real mFrequency;
  Real mOmega;
  Complex mSourceVoltage;
  Real mGridResistance;
  Real mGridInductance;
  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;
  Real mKpPLL;
  Real mKiPLL;
  Real mOmegaCutoff;
  Real mPRef;
  Real mQRef;
  Real mKpPowerCtrl;
  Real mKiPowerCtrl;
  Real mKpCurrCtrl;
  Real mKiCurrCtrl;
};

int main() {
  DPPh1InverterStateSpaceExtractionExample example;
  example.run();
  return 0;
}
