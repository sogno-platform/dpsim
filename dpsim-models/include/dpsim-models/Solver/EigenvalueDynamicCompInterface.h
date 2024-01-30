#pragma once
#include <dpsim-models/Solver/EigenvalueCompInterface.h>

namespace CPS
{
    template <typename VarType>
    /// Interface to be implemented by all dynamic components taking part in eigenvalue extraction.
    class EigenvalueDynamicCompInterface :
        public EigenvalueCompInterface
    {
    public:
        typedef std::shared_ptr<EigenvalueDynamicCompInterface> Ptr;
        typedef std::vector<Ptr> List;

        /// Stamp component into sign matrix used for eigenvalue extraction.
        virtual void stampSignMatrix(MatrixVar<VarType> &signMatrix, Complex coeffDP) = 0;
        /// Stamp component into discretization matrix used for eigenvalue extraction.
        virtual void stampDiscretizationMatrix(MatrixVar<VarType> &discretizationMatrix, Complex coeffDP) = 0;
    };
}