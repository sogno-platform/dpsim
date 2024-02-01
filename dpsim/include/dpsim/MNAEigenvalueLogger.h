#pragma once

#include <dpsim/DataLogger.h>

namespace DPsim
{
	/// Logs eigenvalues during simulation
	class MNAEigenvalueLogger
	{
	public:
		MNAEigenvalueLogger();

		void setLogAttributes(CPS::AttributeStatic<MatrixComp>::Ptr eigenvalues, CPS::AttributeStatic<MatrixComp>::Ptr discreteEigenvalues);

		template <typename VarType>
		void logInitialization(const MatrixVar<VarType> &signMatrix, const MatrixVar<VarType> &discretizationMatrix, const Matrix &branchNodeIncidenceMatrix, const Matrix &nodeBranchIncidenceMatrix);
		template <typename VarType>
		void logExtraction(Real time, Int timeStepCount, const MatrixVar<VarType> &stateMatrix);

	private:
		CPS::Logger::Log mSLog;
		DataLogger mEigenvaluesLogger;
		DataLogger mDiscreteEigenvaluesLogger;
	};
}
