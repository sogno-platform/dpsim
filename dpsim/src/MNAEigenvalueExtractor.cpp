#include <dpsim/MNAEigenvalueExtractor.h>

namespace DPsim
{
    MNAEigenvalueExtractor::MNAEigenvalueExtractor() {}

    MNAEigenvalueExtractor::MNAEigenvalueExtractor(const CPS::SystemTopology &topology, UInt numMatrixNodeIndices)
    {
        initialize(topology, numMatrixNodeIndices);
    }

    void MNAEigenvalueExtractor::initialize(const CPS::SystemTopology &topology, UInt numMatrixNodeIndices)
    {
        identifyEigenvalueComponents(topology.mComponents);
        setBranchIndices();
        createEmptyEigenvalueMatrices(numMatrixNodeIndices);
        stampEigenvalueMatrices();
    }

    void MNAEigenvalueExtractor::identifyEigenvalueComponents(const CPS::IdentifiedObject::List &components)
    {
        // TODO: [Georgii] throw exception if topology contains components that do not implement EigenvalueCompInterface
        for (auto comp : components)
        {
            auto eigenvalueComponent = std::dynamic_pointer_cast<CPS::EigenvalueCompInterface>(comp);
            if (eigenvalueComponent)
            {
                mEigenvalueComponents.push_back(eigenvalueComponent);
            }
        }
    }

    void MNAEigenvalueExtractor::setBranchIndices()
    {
        int size = mEigenvalueComponents.size();
        for (int i = 0; i < size; i++)
        {
            mEigenvalueComponents[i]->setBranchIdx(i);
        }
    }

    void MNAEigenvalueExtractor::createEmptyEigenvalueMatrices(UInt numMatrixNodeIndices)
    {
        int nBranches = mEigenvalueComponents.size();
        mSignMatrix = Matrix(nBranches, nBranches);
        mDiscretizationMatrix = Matrix(nBranches, nBranches);
        mBranchNodeIncidenceMatrix = Matrix(nBranches, numMatrixNodeIndices);
    }

    void MNAEigenvalueExtractor::stampEigenvalueMatrices()
    {
        for (auto comp : mEigenvalueComponents)
        {
            comp->stampEigenvalueMatrices(mSignMatrix, mDiscretizationMatrix, mBranchNodeIncidenceMatrix);
        }
        mNodeBranchIncidenceMatrix = mBranchNodeIncidenceMatrix.transpose();
    }

    void MNAEigenvalueExtractor::extractEigenvalues(const Matrix &powerSystemMatrix, Real timeStep)
    {
        calculateStateMatrix(powerSystemMatrix);
        computeDiscreteEigenvalues();
        recoverEigenvalues(timeStep);
    }

    void MNAEigenvalueExtractor::calculateStateMatrix(const Matrix &powerSystemMatrix)
    {
        // TODO: [Georgii] use back substitution of factorized power system matrix instead of inversion (performance)
        Matrix intermediateResult = powerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
        mStateMatrix = mSignMatrix + mDiscretizationMatrix * mBranchNodeIncidenceMatrix * intermediateResult;
    }

    void MNAEigenvalueExtractor::computeDiscreteEigenvalues()
    {
        auto discreteEigenvaluesIncludingZeros = mStateMatrix.eigenvalues();
        mDiscreteEigenvalues = CPS::Math::returnNonZeroElements(discreteEigenvaluesIncludingZeros);
    }

    void MNAEigenvalueExtractor::recoverEigenvalues(Real timeStep)
    {
        mEigenvalues = 2.0 / timeStep * (mDiscreteEigenvalues.array() - 1.0) / (mDiscreteEigenvalues.array() + 1.0);
    }
}