#pragma once

#include <dpsim/Definitions.h>
#include <dpsim/DataLogger.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim-models/Solver/EigenvalueDynamicCompInterface.h>
#include <dpsim-models/Attribute.h>

namespace DPsim
{
	/// Extracts eigenvalues using MNA power system conductance matrix
	template <typename MatrixType>
	class MNAEigenvalueExtractor
	{
	public:
		MNAEigenvalueExtractor();

		void initialize(const CPS::SystemTopology &topology, UInt numMatrixNodeIndices, Real timeStep);
		void extractEigenvalues(const Matrix &powerSystemMatrix, Real time, Int timeStepCount);		

	private:
		CPS::EigenvalueCompInterface::List mEigenvalueComponents;
		typename CPS::EigenvalueDynamicCompInterface<MatrixType>::List mEigenvalueDynamicComponents;
		Real mTimeStep;
		Real mSystemOmega;
		Complex mCoeffDP;
		MatrixType mSignMatrix;
		MatrixType mDiscretizationMatrix;
		Matrix mBranchNodeIncidenceMatrix;
		Matrix mNodeBranchIncidenceMatrix;
		MatrixType mStateMatrix;
		CPS::AttributeStatic<MatrixComp>::Ptr mDiscreteEigenvalues;
		CPS::AttributeStatic<MatrixComp>::Ptr mEigenvalues;
		CPS::Logger::Log mSLog;
		DataLogger mEigenvaluesLogger;
		DataLogger mDiscreteEigenvaluesLogger;

		void setParameters(const CPS::SystemTopology &topology, Real timeStep);
		void identifyEigenvalueComponents(const CPS::IdentifiedObject::List &components);
		void setBranchIndices();
		void createEmptyEigenvalueMatrices(UInt numMatrixNodeIndices);
		void stampEigenvalueMatrices();
		void setLogAttributes();
		void logInitialization();
		void calculateStateMatrix(const Matrix &powerSystemMatrix);
		void computeDiscreteEigenvalues();
		void recoverEigenvalues();
		void logExtraction(Real time, Int timeStepCount);
	};
}
