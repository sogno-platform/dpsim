#include <dpsim/MNAEigenvalueLogger.h>

namespace DPsim {
MNAEigenvalueLogger::MNAEigenvalueLogger(String name,
                                         CPS::Logger::Level logLevel)
    : mSLog(CPS::Logger::get(name, logLevel, logLevel)),
      mEigenvaluesLogger("eigenvalues", true, 1),
      mDiscreteEigenvaluesLogger("discreteEigenvalues", true, 1) {}

void MNAEigenvalueLogger::setLogAttributes(
    CPS::AttributeStatic<MatrixComp>::Ptr eigenvalues,
    CPS::AttributeStatic<MatrixComp>::Ptr discreteEigenvalues) {
  mEigenvaluesLogger.logAttribute("eigenvalues", eigenvalues);
  mDiscreteEigenvaluesLogger.logAttribute("discreteEigenvalues",
                                          discreteEigenvalues);
}

template <typename VarType>
void MNAEigenvalueLogger::logInitialization(
    const MatrixVar<VarType> &signMatrix,
    const MatrixVar<VarType> &discretizationMatrix,
    const Matrix &branchNodeIncidenceMatrix,
    const Matrix &nodeBranchIncidenceMatrix) {
  SPDLOG_LOGGER_DEBUG(mSLog, "---- Initialize ----");
  SPDLOG_LOGGER_DEBUG(mSLog, "sign matrix: {}",
                      CPS::Logger::matrixVarToString(signMatrix));
  SPDLOG_LOGGER_DEBUG(mSLog, "discretization matrix: {}",
                      CPS::Logger::matrixVarToString(discretizationMatrix));
  SPDLOG_LOGGER_DEBUG(mSLog, "branch <-> node incidence matrix: {}",
                      CPS::Logger::matrixToString(branchNodeIncidenceMatrix));
  SPDLOG_LOGGER_DEBUG(mSLog, "node <-> branch incidence matrix: {}",
                      CPS::Logger::matrixToString(nodeBranchIncidenceMatrix));
}
template void MNAEigenvalueLogger::logInitialization<Real>(
    const MatrixVar<Real> &signMatrix,
    const MatrixVar<Real> &discretizationMatrix,
    const Matrix &branchNodeIncidenceMatrix,
    const Matrix &nodeBranchIncidenceMatrix);
template void MNAEigenvalueLogger::logInitialization<Complex>(
    const MatrixVar<Complex> &signMatrix,
    const MatrixVar<Complex> &discretizationMatrix,
    const Matrix &branchNodeIncidenceMatrix,
    const Matrix &nodeBranchIncidenceMatrix);

template <typename VarType>
void MNAEigenvalueLogger::logExtraction(Real time, Int timeStepCount,
                                        const MatrixVar<VarType> &stateMatrix) {
  SPDLOG_LOGGER_DEBUG(mSLog, "---- Extract eigenvalues ----");
  SPDLOG_LOGGER_DEBUG(mSLog, "time: {}", CPS::Logger::realToString(time));
  SPDLOG_LOGGER_DEBUG(mSLog, "discretized state matrix: {}",
                      CPS::Logger::matrixVarToString(stateMatrix));

  mEigenvaluesLogger.log(time, timeStepCount);
  mDiscreteEigenvaluesLogger.log(time, timeStepCount);
}
template void
MNAEigenvalueLogger::logExtraction<Real>(Real time, Int timeStepCount,
                                         const MatrixVar<Real> &stateMatrix);
template void MNAEigenvalueLogger::logExtraction<Complex>(
    Real time, Int timeStepCount, const MatrixVar<Complex> &stateMatrix);

void MNAEigenvalueLogger::close() {
  mEigenvaluesLogger.close();
  mDiscreteEigenvaluesLogger.close();
  mSLog->flush();
}
} // namespace DPsim
