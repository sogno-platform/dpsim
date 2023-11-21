#pragma once

#include <dpsim/Definitions.h>
#include <dpsim/DataLogger.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim-models/Solver/EigenvalueCompInterface.h>

namespace DPsim
{
	/// Extracts eigenvalues using MNA power system conductance matrix
	class MNAEigenvalueExtractor
	{
	public:
		MNAEigenvalueExtractor();
		MNAEigenvalueExtractor(const CPS::SystemTopology &topology, UInt numMatrixNodeIndices);

		void initialize(const CPS::SystemTopology &topology, UInt numMatrixNodeIndices);
		void extractEigenvalues(const Matrix &powerSystemMatrix, Real timeStep);		

	private:
		CPS::EigenvalueCompInterface::List mEigenvalueComponents;
		Matrix mSignMatrix;
		Matrix mDiscretizationMatrix;
		Matrix mBranchNodeIncidenceMatrix;
		Matrix mNodeBranchIncidenceMatrix;
		Matrix mStateMatrix;
		MatrixComp mDiscreteEigenvalues;
		MatrixComp mEigenvalues;
		CPS::Logger::Log mSLog;

		void identifyEigenvalueComponents(const CPS::IdentifiedObject::List &components);
		void setBranchIndices();
		void createEmptyEigenvalueMatrices(UInt numMatrixNodeIndices);
		void stampEigenvalueMatrices();
		void calculateStateMatrix(const Matrix &powerSystemMatrix);
		void computeDiscreteEigenvalues();
		void recoverEigenvalues(Real timeStep);
	};
}
