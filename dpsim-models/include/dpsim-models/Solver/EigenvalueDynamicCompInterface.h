#pragma once
#include <dpsim-models/Solver/EigenvalueCompInterface.h>

namespace CPS
{
    template <typename MatrixType>
    /// Interface to be implemented by all dynamic components taking part in eigenvalue extraction.
    class EigenvalueDynamicCompInterface :
        public EigenvalueCompInterface
    {
    public:
        typedef std::shared_ptr<EigenvalueDynamicCompInterface> Ptr;
        typedef std::vector<Ptr> List;

        /// Stamp component into sign matrix used for eigenvalue extraction.
        virtual void stampSignMatrix(MatrixType &signMatrix, Complex coeffDP) = 0;
        /// Stamp component into discretization matrix used for eigenvalue extraction.
        virtual void stampDiscretizationMatrix(MatrixType &discretizationMatrix, Complex coeffDP) = 0;
    };
}