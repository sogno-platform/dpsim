#pragma once

#include <dpsim-models/Solver/EigenvalueDynamicCompInterface.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim/MNAEigenvalueLogger.h>

namespace DPsim {
/// Extracts eigenvalues using MNA power system conductance matrix
template <typename VarType> class MNAEigenvalueExtractor {
public:
  MNAEigenvalueExtractor(CPS::Logger::Level logLevel);

  void initialize(const CPS::SystemTopology &topology,
                  UInt numMatrixNodeIndices, Real timeStep);
  void extractEigenvalues(const Matrix &powerSystemMatrix, Real time,
                          Int timeStepCount);
  void closeLogger();

private:
  CPS::EigenvalueCompInterface::List mEigenvalueComponents;
  typename CPS::EigenvalueDynamicCompInterface<VarType>::List
      mEigenvalueDynamicComponents;
  Real mTimeStep;
  Real mSystemOmega;
  Complex mCoeffDP;
  MatrixVar<VarType> mSignMatrix;
  MatrixVar<VarType> mDiscretizationMatrix;
  Matrix mBranchNodeIncidenceMatrix;
  Matrix mNodeBranchIncidenceMatrix;
  MatrixVar<VarType> mStateMatrix;
  CPS::AttributeStatic<MatrixComp>::Ptr mDiscreteEigenvalues;
  CPS::AttributeStatic<MatrixComp>::Ptr mEigenvalues;
  MNAEigenvalueLogger mLogger;

  void setParameters(const CPS::SystemTopology &topology, Real timeStep);
  void
  identifyEigenvalueComponents(const CPS::IdentifiedObject::List &components);
  void setBranchIndices();
  void createEmptyEigenvalueMatrices(UInt numMatrixNodeIndices);
  void stampEigenvalueMatrices();
  void calculateStateMatrix(const Matrix &powerSystemMatrix);
  void computeDiscreteEigenvalues();
  void recoverEigenvalues();
};
} // namespace DPsim
