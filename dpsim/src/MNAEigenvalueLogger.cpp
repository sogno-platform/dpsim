#include <dpsim/MNAEigenvalueLogger.h>

namespace DPsim
{
    MNAEigenvalueLogger::MNAEigenvalueLogger() : mSLog(CPS::Logger::get("MNAEigenvalueExtractor", CPS::Logger::Level::info, CPS::Logger::Level::info)),
                                                 mEigenvaluesLogger("eigenvalues", true, 1), mDiscreteEigenvaluesLogger("discreteEigenvalues", true, 1)
    {
    }

    void MNAEigenvalueLogger::setLogAttributes(CPS::AttributeStatic<MatrixComp>::Ptr eigenvalues, CPS::AttributeStatic<MatrixComp>::Ptr discreteEigenvalues)
    {
        mEigenvaluesLogger.logAttribute("eigenvalues", eigenvalues);
        mDiscreteEigenvaluesLogger.logAttribute("discreteEigenvalues", discreteEigenvalues);
    }

    template <typename VarType>
    void MNAEigenvalueLogger::logInitialization(const MatrixVar<VarType> &signMatrix, const MatrixVar<VarType> &discretizationMatrix, const Matrix &branchNodeIncidenceMatrix, const Matrix &nodeBranchIncidenceMatrix)
    {
        SPDLOG_LOGGER_INFO(mSLog, "---- Initialize ----");
        SPDLOG_LOGGER_INFO(mSLog, "sign matrix: {}", CPS::Logger::matrixVarToString(signMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "discretization matrix: {}", CPS::Logger::matrixVarToString(discretizationMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "branch <-> node incidence matrix: {}", CPS::Logger::matrixToString(branchNodeIncidenceMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "node <-> branch incidence matrix: {}", CPS::Logger::matrixToString(nodeBranchIncidenceMatrix));
        mSLog->flush();
    }
    template void MNAEigenvalueLogger::logInitialization<Real>(const MatrixVar<Real> &signMatrix, const MatrixVar<Real> &discretizationMatrix, const Matrix &branchNodeIncidenceMatrix, const Matrix &nodeBranchIncidenceMatrix);
    template void MNAEigenvalueLogger::logInitialization<Complex>(const MatrixVar<Complex> &signMatrix, const MatrixVar<Complex> &discretizationMatrix, const Matrix &branchNodeIncidenceMatrix, const Matrix &nodeBranchIncidenceMatrix);

    template <typename VarType>
    void MNAEigenvalueLogger::logExtraction(Real time, Int timeStepCount, const MatrixVar<VarType> &stateMatrix)
    {
        SPDLOG_LOGGER_INFO(mSLog, "---- Extract eigenvalues ----");
        SPDLOG_LOGGER_INFO(mSLog, "time: {}", CPS::Logger::realToString(time));
        SPDLOG_LOGGER_INFO(mSLog, "discretized state matrix: {}", CPS::Logger::matrixVarToString(stateMatrix));
        mSLog->flush();

        mEigenvaluesLogger.log(time, timeStepCount);
        mDiscreteEigenvaluesLogger.log(time, timeStepCount);
    }
    template void MNAEigenvalueLogger::logExtraction<Real>(Real time, Int timeStepCount, const MatrixVar<Real> &stateMatrix);
    template void MNAEigenvalueLogger::logExtraction<Complex>(Real time, Int timeStepCount, const MatrixVar<Complex> &stateMatrix);
}
