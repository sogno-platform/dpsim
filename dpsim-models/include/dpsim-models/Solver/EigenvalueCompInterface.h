
#pragma once
#include <dpsim-models/Definitions.h>

namespace CPS {
    /// Interface to be implemented by all components taking part in eigenvalue extraction.
    class EigenvalueCompInterface
    {
    public:
        typedef std::shared_ptr<EigenvalueCompInterface> Ptr;
        typedef std::vector<Ptr> List;
        /// Branch index
        UInt mBranchIdx;

        /// Stamp component into matrices used for eigenvalue extraction.
        virtual void stampEigenvalueMatrices(Matrix& signMatrix, Matrix& discretizationMatrix, Matrix& branchNodeIncidenceMatrix) = 0;
        /// Set component's branch index.
        virtual void setBranchIdx(int i) = 0;
    };
}