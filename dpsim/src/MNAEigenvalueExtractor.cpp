#include <dpsim/MNAEigenvalueExtractor.h>

namespace DPsim
{
    template <typename MatrixType>
    MNAEigenvalueExtractor<MatrixType>::MNAEigenvalueExtractor() : mSLog(CPS::Logger::get("MNAEigenvalueExtractor", CPS::Logger::Level::info, CPS::Logger::Level::info)),
                                                                   mEigenvaluesLogger("eigenvalues", true, 1), mDiscreteEigenvaluesLogger("discreteEigenvalues", true, 1)
    {
        mEigenvalues = CPS::AttributeStatic<MatrixComp>::make();
        mDiscreteEigenvalues = CPS::AttributeStatic<MatrixComp>::make();
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::initialize(const CPS::SystemTopology &topology, UInt numMatrixNodeIndices, Real timeStep)
    {
        setParameters(topology, timeStep);
        identifyEigenvalueComponents(topology.mComponents);
        setBranchIndices();
        createEmptyEigenvalueMatrices(numMatrixNodeIndices);
        stampEigenvalueMatrices();
        setLogAttributes();
        logInitialization();
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::setParameters(const CPS::SystemTopology &topology, Real timeStep)
    {
        mTimeStep = timeStep;
        // Relevant only for DP
        mSystemOmega = topology.mSystemOmega;

        Real denominator = mTimeStep * mTimeStep * mSystemOmega * mSystemOmega + 4.0;
        Real realPart = (4.0 - mTimeStep * mTimeStep * mSystemOmega * mSystemOmega) / denominator;
        Real imagPart = (-4.0 * mTimeStep * mSystemOmega) / denominator;
        mCoeffDP = Complex(realPart, imagPart);
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::identifyEigenvalueComponents(const CPS::IdentifiedObject::List &components)
    {
        // TODO: [Georgii] throw exception if topology contains components that do not implement EigenvalueCompInterface
        for (auto comp : components)
        {
            auto eigenvalueComponent = std::dynamic_pointer_cast<CPS::EigenvalueCompInterface>(comp);
            if (eigenvalueComponent)
            {
                mEigenvalueComponents.push_back(eigenvalueComponent);
                auto eigenvalueDynamicComponent = std::dynamic_pointer_cast<CPS::EigenvalueDynamicCompInterface<MatrixType>>(comp);
                if (eigenvalueDynamicComponent)
                {
                    mEigenvalueDynamicComponents.push_back(eigenvalueDynamicComponent);
                }
            }
        }
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::setBranchIndices()
    {
        int size = mEigenvalueComponents.size();
        for (int i = 0; i < size; i++)
        {
            mEigenvalueComponents[i]->setBranchIdx(i);
        }
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::createEmptyEigenvalueMatrices(UInt numMatrixNodeIndices)
    {
        int nBranches = mEigenvalueComponents.size();
        // TODO: [Georgii] use Matrix::Zero to initialize with zeros
        mSignMatrix = Matrix(nBranches, nBranches);
        mDiscretizationMatrix = Matrix(nBranches, nBranches);
        mBranchNodeIncidenceMatrix = Matrix(nBranches, numMatrixNodeIndices);
        **mEigenvalues = MatrixComp(mEigenvalueDynamicComponents.size(), 1);
        **mDiscreteEigenvalues = MatrixComp(mEigenvalueDynamicComponents.size(), 1);
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::stampEigenvalueMatrices()
    {
        for (auto comp : mEigenvalueComponents)
        {
            comp->stampBranchNodeIncidenceMatrix(mBranchNodeIncidenceMatrix);
        }
        mNodeBranchIncidenceMatrix = mBranchNodeIncidenceMatrix.transpose();
        for (auto dynamicComp : mEigenvalueDynamicComponents)
        {
            dynamicComp->stampSignMatrix(mSignMatrix, mCoeffDP);
            dynamicComp->stampDiscretizationMatrix(mDiscretizationMatrix, mCoeffDP);
        }
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::setLogAttributes()
    {
        mEigenvaluesLogger.logAttribute("eigenvalues", mEigenvalues);
        mDiscreteEigenvaluesLogger.logAttribute("discreteEigenvalues", mDiscreteEigenvalues);
    }

    template <>
    void MNAEigenvalueExtractor<Matrix>::logInitialization()
    {
        SPDLOG_LOGGER_INFO(mSLog, "---- Initialize ----");
        SPDLOG_LOGGER_INFO(mSLog, "sign matrix: {}", CPS::Logger::matrixToString(mSignMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "discretization matrix: {}", CPS::Logger::matrixToString(mDiscretizationMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "branch <-> node incidence matrix: {}", CPS::Logger::matrixToString(mBranchNodeIncidenceMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "node <-> branch incidence matrix: {}", CPS::Logger::matrixToString(mNodeBranchIncidenceMatrix));
        mSLog->flush();
    }

    template <>
    void MNAEigenvalueExtractor<MatrixComp>::logInitialization()
    {
        SPDLOG_LOGGER_INFO(mSLog, "---- Initialize ----");
        SPDLOG_LOGGER_INFO(mSLog, "sign matrix: {}", CPS::Logger::matrixCompToString(mSignMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "discretization matrix: {}", CPS::Logger::matrixCompToString(mDiscretizationMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "branch <-> node incidence matrix: {}", CPS::Logger::matrixToString(mBranchNodeIncidenceMatrix));
        SPDLOG_LOGGER_INFO(mSLog, "node <-> branch incidence matrix: {}", CPS::Logger::matrixToString(mNodeBranchIncidenceMatrix));
        mSLog->flush();
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::extractEigenvalues(const Matrix &powerSystemMatrix, Real time, Int timeStepCount)
    {
        calculateStateMatrix(powerSystemMatrix);
        computeDiscreteEigenvalues();
        recoverEigenvalues();
        logExtraction(time, timeStepCount);
    }

    template <>
    void MNAEigenvalueExtractor<Matrix>::calculateStateMatrix(const Matrix &powerSystemMatrix)
    {
        // TODO: [Georgii] use back substitution of factorized power system matrix instead of inversion (performance)
        Matrix intermediateResult = powerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
        mStateMatrix = mSignMatrix + mDiscretizationMatrix * mBranchNodeIncidenceMatrix * intermediateResult;
    }

    template <>
    void MNAEigenvalueExtractor<MatrixComp>::calculateStateMatrix(const Matrix &powerSystemMatrix)
    {
        // TODO: [Georgii] use back substitution of factorized power system matrix instead of inversion (performance)
        MatrixComp compPowerSystemMatrix = CPS::Math::convertToComplex(powerSystemMatrix);
        MatrixComp intermediateResult = compPowerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
        mStateMatrix = mSignMatrix + mDiscretizationMatrix * mBranchNodeIncidenceMatrix * intermediateResult;
    }

    template <typename MatrixType>
    void MNAEigenvalueExtractor<MatrixType>::computeDiscreteEigenvalues()
    {
        auto discreteEigenvaluesIncludingZeros = mStateMatrix.eigenvalues();
        **mDiscreteEigenvalues = CPS::Math::returnNonZeroElements(discreteEigenvaluesIncludingZeros);
        // TODO: [Georgii] filter out eigenvalues = -1 + 0i to avoid division by zero in recoverEigenvalues()
    }

    template <>
    void MNAEigenvalueExtractor<Matrix>::recoverEigenvalues()
    {
        **mEigenvalues = 2.0 / mTimeStep * ((**mDiscreteEigenvalues).array() - 1.0) / ((**mDiscreteEigenvalues).array() + 1.0);
    }

    template <>
    void MNAEigenvalueExtractor<MatrixComp>::recoverEigenvalues()
    {
        // TODO: [Georgii] use Complex(0,1) instead of j
        **mEigenvalues = 2.0 / mTimeStep * ((**mDiscreteEigenvalues).array() - 1.0) / ((**mDiscreteEigenvalues).array() + 1.0) + 1.0j * mSystemOmega;
    }

    template <>
    void MNAEigenvalueExtractor<Matrix>::logExtraction(Real time, Int timeStepCount)
    {
        SPDLOG_LOGGER_INFO(mSLog, "---- Extract eigenvalues ----");
        SPDLOG_LOGGER_INFO(mSLog, "time: {}", CPS::Logger::realToString(time));
        SPDLOG_LOGGER_INFO(mSLog, "discretized state matrix: {}", CPS::Logger::matrixToString(mStateMatrix));
        mSLog->flush();

        mEigenvaluesLogger.log(time, timeStepCount);
        mDiscreteEigenvaluesLogger.log(time, timeStepCount);
    }

    template <>
    void MNAEigenvalueExtractor<MatrixComp>::logExtraction(Real time, Int timeStepCount)
    {
        SPDLOG_LOGGER_INFO(mSLog, "---- Extract eigenvalues ----");
        SPDLOG_LOGGER_INFO(mSLog, "time: {}", CPS::Logger::realToString(time));
        SPDLOG_LOGGER_INFO(mSLog, "discretized state matrix: {}", CPS::Logger::matrixCompToString(mStateMatrix));
        mSLog->flush();

        mEigenvaluesLogger.log(time, timeStepCount);
        mDiscreteEigenvaluesLogger.log(time, timeStepCount);
    }

    template class MNAEigenvalueExtractor<Matrix>;
    template class MNAEigenvalueExtractor<MatrixComp>;
}
