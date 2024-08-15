/* Voltage Source that reads references from a file and replays them.
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Signal/CosineFMGenerator.h>
#include <dpsim-models/Signal/DCGenerator.h>
#include <dpsim-models/Signal/FrequencyRampGenerator.h>
#include <dpsim-models/Signal/SignalGenerator.h>
#include <dpsim-models/Signal/SineWaveGenerator.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <filesystem>
#include <villas/formats/protobuf.hpp>
#include <villas/sample.hpp>

namespace CPS {
namespace DP {
namespace Ph1 {

class ProfileVoltageSource : public MNASimPowerComp<Complex>, public DAEInterface, public SharedFactory<ProfileVoltageSource> {
private:
  ///
  void updateVoltage(Real time);
  ///
  // CPS::Signal::SignalGenerator::Ptr mSrcSig;
  std::filesystem::path mSourceFile;
  size_t mSourceIndex = 0;
  std::vector<double> mSamples;

  void readFromFile();

public:
  const CPS::Attribute<Complex>::Ptr mVoltage;

  /// Defines UID, name, component parameters and logging level
  ProfileVoltageSource(String uid, String name, std::filesystem::path sourceFile, Logger::Level loglevel = Logger::Level::off);
  /// Defines UID, name, component parameters and logging level
  ProfileVoltageSource(String name, std::filesystem::path sourceFile, Logger::Level logLevel = Logger::Level::off) : ProfileVoltageSource(name, name, sourceFile, logLevel) {}
  ///
  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  ///
  void setSourceFile(std::filesystem::path file, size_t index = 0);

  // #### MNA Section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix, Int freqIdx) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompApplyRightSideVectorStampHarm(Matrix &rightVector) override;
  /// Returns current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
                                      Attribute<Matrix>::Ptr &leftVector) override;

  class MnaPreStepHarm : public CPS::Task {
  public:
    MnaPreStepHarm(ProfileVoltageSource &voltageSource) : Task(**voltageSource.mName + ".MnaPreStepHarm"), mVoltageSource(voltageSource) {
      mAttributeDependencies.push_back(mVoltageSource.mVoltage);
      mModifiedAttributes.push_back(mVoltageSource.mRightVector);
      mModifiedAttributes.push_back(mVoltageSource.mIntfVoltage);
    }
    void execute(Real time, Int timeStepCount);

  private:
    ProfileVoltageSource &mVoltageSource;
  };

  class MnaPostStepHarm : public CPS::Task {
  public:
    MnaPostStepHarm(ProfileVoltageSource &voltageSource, const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**voltageSource.mName + ".MnaPostStepHarm"), mVoltageSource(voltageSource), mLeftVectors(leftVectors) {
      for (UInt i = 0; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
      mModifiedAttributes.push_back(mVoltageSource.mIntfCurrent);
    }
    void execute(Real time, Int timeStepCount);

  private:
    ProfileVoltageSource &mVoltageSource;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };

  // #### DAE Section ####
  /// Residual function for DAE Solver
  void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int> &off) override;
  /// Voltage Getter
  Complex daeInitialize() override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
