/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/SP/SP_Ph1_RXLine.h"

using namespace CPS;

SP::Ph1::RXLine::RXLine(String uid, String name, Real baseVoltage,
                        Real resistance, Real inductance,
                        Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes), CompositePowerComp<Complex>(
                                          uid, name, true, true, logLevel),
      mBaseVoltage(mAttributes->create<Real>("base_Voltage", baseVoltage)),
      mInductance(mAttributes->create<Real>("L_series")),
      mActivePowerInjection(mAttributes->create<Real>("p_inj")),
      mReactivePowerInjection(mAttributes->create<Real>("q_inj")),
      mCurrent(mAttributes->create<MatrixComp>("current_vector")),
      mActivePowerBranch(mAttributes->create<Matrix>("p_branch_vector")),
      mReactivePowerBranch(mAttributes->create<Matrix>("q_branch_vector")) {

  setTerminalNumber(2);

  **mSeriesRes = resistance;
  **mInductance = inductance;

  **mCurrent = MatrixComp::Zero(2, 1);
  **mActivePowerBranch = Matrix::Zero(2, 1);
  **mReactivePowerBranch = Matrix::Zero(2, 1);
  // mLog.Log(Logger::Level::DEBUG) << "Create " << this->type() << " " << name
  // 	<< " R=" << resistance << " L=" << inductance
  // 	 << std::endl;
}

SP::Ph1::RXLine::RXLine(String uid, String name, Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes), CompositePowerComp<Complex>(
                                          uid, name, true, true, logLevel),
      mBaseVoltage(mAttributes->create<Real>("base_Voltage")),
      mInductance(mAttributes->create<Real>("L_series")),
      mActivePowerInjection(mAttributes->create<Real>("p_inj")),
      mReactivePowerInjection(mAttributes->create<Real>("q_inj")),
      mCurrent(mAttributes->create<MatrixComp>("current_vector")),
      mActivePowerBranch(mAttributes->create<Matrix>("p_branch_vector")),
      mReactivePowerBranch(mAttributes->create<Matrix>("q_branch_vector")) {

  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

void SP::Ph1::RXLine::setPerUnitSystem(Real baseApparentPower, Real baseOmega) {
  mBaseApparentPower = baseApparentPower;
  mBaseOmega = baseOmega;
  mBaseImpedance = (**mBaseVoltage * **mBaseVoltage) / mBaseApparentPower;
  mBaseAdmittance = 1.0 / mBaseImpedance;
  mBaseInductance = mBaseImpedance / mBaseOmega;
  /// I_base = S_base / V_line
  mBaseCurrent = baseApparentPower / (**mBaseVoltage * sqrt(3));
  /*
	mLog.Log(Logger::Level::INFO) << "#### Set Per Unit System for " << **mName << std::endl;
	mLog.Log(Logger::Level::INFO) << " Base Voltage= " << mBaseVoltage << " [V] " << " Base Impedance= " << mBaseImpedance << " [Ohm] " << std::endl;
	*/

  mSeriesResPerUnit = **mSeriesRes / mBaseImpedance;
  mSeriesIndPerUnit = **mInductance / mBaseInductance;
  /*
	mLog.Log(Logger::Level::INFO) << "Series Resistance Per Unit= " << " " << mSeriesResPerUnit << " [Ohm] "
		<< " Series Inductance Per Unit= " << " " << mSeriesIndPerUnit << " [H] "
		<< std::endl;
    mLog.Log(Logger::Level::INFO)  << "r " << mSeriesResPerUnit << std::endl << "	x: " << mBaseOmega * mInductance / mBaseImpedance<<std::endl;*/
}

void SP::Ph1::RXLine::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) {
  updateMatrixNodeIndices();
  int bus1 = this->matrixNodeIndex(0);
  int bus2 = this->matrixNodeIndex(1);

  //dimension check
  /* TODO: FIX
	if (bus1 > (n - 1) || bus2 > (n - 1)) {
		std::stringstream ss;
		ss << "Line>>" << this->getName() << ": Wrong Y dimension: " << n;
		throw std::invalid_argument(ss.str());
		//std::cout << "Line>>" << Name << ": Wrong Y dimension: " << n << endl;
		return;
	}
	*/

  //create the element admittance matrix
  Complex y =
      Complex(1, 0) / Complex(mSeriesResPerUnit, 1. * mSeriesIndPerUnit);

  //Fill the internal matrix
  mY_element = MatrixComp(2, 2);
  mY_element(0, 0) = y;
  mY_element(0, 1) = -y;
  mY_element(1, 0) = -y;
  mY_element(1, 1) = y;

  //check for inf or nan
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      if (std::isinf(mY_element.coeff(i, j).real()) ||
          std::isinf(mY_element.coeff(i, j).imag())) {
        std::cout << mY_element << std::endl;
        std::stringstream ss;
        ss << "Line>>" << this->name()
           << ": infinite or nan values in the element Y at: " << i << "," << j;
        throw std::invalid_argument(ss.str());
        std::cout << "Line>>" << this->name()
                  << ": infinite or nan values in the element Y at: " << i
                  << "," << j << std::endl;
      }

  //set the circuit matrix values
  Y.coeffRef(bus1, bus1) += mY_element.coeff(0, 0);
  Y.coeffRef(bus1, bus2) += mY_element.coeff(0, 1);
  Y.coeffRef(bus2, bus1) += mY_element.coeff(1, 0);
  Y.coeffRef(bus2, bus2) += mY_element.coeff(1, 1);

  //mLog.Log(Logger::Level::INFO) << "#### Y matrix stamping: " << std::endl;
  //mLog.Log(Logger::Level::INFO) << mY_element << std::endl;
}

void SP::Ph1::RXLine::updateBranchFlow(VectorComp &current,
                                       VectorComp &powerflow) {
  **mCurrent = current * mBaseCurrent;
  **mActivePowerBranch = powerflow.real() * mBaseApparentPower;
  **mReactivePowerBranch = powerflow.imag() * mBaseApparentPower;
}

void SP::Ph1::RXLine::storeNodalInjection(Complex powerInjection) {
  **mActivePowerInjection = std::real(powerInjection) * mBaseApparentPower;
  **mReactivePowerInjection = std::imag(powerInjection) * mBaseApparentPower;
}

MatrixComp SP::Ph1::RXLine::Y_element() { return mY_element; }

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr SP::Ph1::RXLine::clone(String name) {
  auto copy = RXLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd);
  return copy;
}

void SP::Ph1::RXLine::initializeFromNodesAndTerminals(Real frequency) {

  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  Complex impedance = {**mSeriesRes, **mSeriesInd * 2. * PI * frequency};
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;
  mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) +
                                      (**mIntfCurrent)(0, 0) * **mSeriesRes);

  // Default model with virtual node in between
  mSubResistor =
      std::make_shared<SP::Ph1::Resistor>(**mName + "_res", mLogLevel);
  mSubResistor->setParameters(**mSeriesRes);
  mSubResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  mSubResistor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor =
      std::make_shared<SP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(**mSeriesInd);
  mSubInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  mSubInductor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mInitialResistor =
      std::make_shared<SP::Ph1::Resistor>(**mName + "_snubber_res", mLogLevel);
  mInitialResistor->setParameters(1e6);
  mInitialResistor->connect({SimNode::GND, mTerminals[1]->node()});
  mInitialResistor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mInitialResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void SP::Ph1::RXLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
};

void SP::Ph1::RXLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
};

void SP::Ph1::RXLine::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::RXLine::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph1::RXLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::RXLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mSubInductor->intfCurrent()(0, 0);
}
