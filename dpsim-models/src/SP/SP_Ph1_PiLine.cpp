/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_PiLine.h>

using namespace CPS;

SP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes),
      CompositePowerComp<Complex>(uid, name, false, true, logLevel),
      mBaseVoltage(mAttributes->create<Real>("base_Voltage")),
      mCurrent(mAttributes->create<MatrixComp>("current_vector")),
      mActivePowerBranch(mAttributes->create<Matrix>("p_branch_vector")),
      mReactivePowerBranch(mAttributes->create<Matrix>("q_branch_vector")),
      mActivePowerInjection(mAttributes->create<Real>("p_inj")),
      mReactivePowerInjection(mAttributes->create<Real>("q_inj")) {

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  mSLog->flush();

  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
  **mCurrent = MatrixComp::Zero(2, 1);
  **mActivePowerBranch = Matrix::Zero(2, 1);
  **mReactivePowerBranch = Matrix::Zero(2, 1);
}

void SP::Ph1::PiLine::setParameters(Real resistance, Real inductance,
                                    Real capacitance, Real conductance) {

  **mSeriesRes = resistance;
  **mSeriesInd = inductance;
  SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [H]",
                     **mSeriesRes, **mSeriesInd);

  if (capacitance > 0) {
    **mParallelCap = capacitance;
  } else {
    **mParallelCap = 1e-12;
    SPDLOG_LOGGER_WARN(
        mSLog, "Zero value for Capacitance, setting default value of C={} [F]",
        **mParallelCap);
  }
  if (conductance > 0) {
    **mParallelCond = conductance;
  } else {
    if (mBehaviour == Behaviour::Initialization)
      **mParallelCond =
          (conductance >= 0)
              ? conductance
              : 1e-6; // init mode for initFromPowerFlow of mna system components
    else
      **mParallelCond = (conductance > 0) ? conductance : 1e-6;
    SPDLOG_LOGGER_WARN(
        mSLog, "Zero value for Conductance, setting default value of G={} [S]",
        **mParallelCond);
  }
  SPDLOG_LOGGER_INFO(mSLog, "Capacitance={} [F] Conductance={} [S]",
                     **mParallelCap, **mParallelCond);
  mSLog->flush();
  mParametersSet = true;
}

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr SP::Ph1::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

// #### Powerflow section ####
void SP::Ph1::PiLine::setBaseVoltage(Real baseVoltage) {
  **mBaseVoltage = baseVoltage;
}

void SP::Ph1::PiLine::calculatePerUnitParameters(Real baseApparentPower,
                                                 Real baseOmega) {
  SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}",
                     **mName);
  mBaseApparentPower = baseApparentPower;
  mBaseOmega = baseOmega;
  SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]",
                     baseApparentPower, baseOmega);

  mBaseImpedance = (**mBaseVoltage * **mBaseVoltage) / mBaseApparentPower;
  mBaseAdmittance = 1.0 / mBaseImpedance;
  mBaseInductance = mBaseImpedance / mBaseOmega;
  mBaseCapacitance = 1.0 / mBaseOmega / mBaseImpedance;
  mBaseCurrent = baseApparentPower /
                 (**mBaseVoltage *
                  sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
  SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Impedance={} [Ohm]",
                     **mBaseVoltage, mBaseImpedance);

  mSeriesResPerUnit = **mSeriesRes / mBaseImpedance;
  mSeriesIndPerUnit = **mSeriesInd / mBaseInductance;
  mParallelCapPerUnit = **mParallelCap / mBaseCapacitance;
  mParallelCondPerUnit = **mParallelCond / mBaseAdmittance;

  SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [pu] Reactance={} [pu]",
                     mSeriesResPerUnit, 1. * mSeriesIndPerUnit);
  SPDLOG_LOGGER_INFO(mSLog, "Susceptance={} [pu] Conductance={} [pu]",
                     1. * mParallelCapPerUnit, mParallelCondPerUnit);
  mSLog->flush();
}

void SP::Ph1::PiLine::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) {
  int bus1 = this->matrixNodeIndex(0);
  int bus2 = this->matrixNodeIndex(1);

  //create the element admittance matrix
  Complex y =
      Complex(1, 0) / Complex(mSeriesResPerUnit, 1. * mSeriesIndPerUnit);
  Complex ys =
      Complex(mParallelCondPerUnit, 1. * mParallelCapPerUnit) / Complex(2, 0);

  //Fill the internal matrix
  mY_element = MatrixComp(2, 2);
  mY_element(0, 0) = y + ys;
  mY_element(0, 1) = -y;
  mY_element(1, 0) = -y;
  mY_element(1, 1) = y + ys;

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
  Y.coeffRef(bus2, bus2) += mY_element.coeff(1, 1);
  Y.coeffRef(bus2, bus1) += mY_element.coeff(1, 0);

  SPDLOG_LOGGER_INFO(mSLog, "#### PF Y matrix stamping #### ");
  SPDLOG_LOGGER_INFO(mSLog, "{}", mY_element);
  mSLog->flush();
}

void SP::Ph1::PiLine::updateBranchFlow(VectorComp &current,
                                       VectorComp &powerflow) {
  **mCurrent = current * mBaseCurrent;
  **mActivePowerBranch = powerflow.real() * mBaseApparentPower;
  **mReactivePowerBranch = powerflow.imag() * mBaseApparentPower;
}

void SP::Ph1::PiLine::storeNodalInjection(Complex powerInjection) {
  **mActivePowerInjection = std::real(powerInjection) * mBaseApparentPower;
  **mReactivePowerInjection = std::imag(powerInjection) * mBaseApparentPower;
}

MatrixComp SP::Ph1::PiLine::Y_element() { return mY_element; }

void SP::Ph1::PiLine::initializeFromNodesAndTerminals(Real frequency) {

  // By default there is always a small conductance to ground to
  // avoid problems with floating nodes.
  **mParallelCond = (**mParallelCond >= 0) ? **mParallelCond : 1e-6;

  // Static calculation
  Real omega = 2. * PI * frequency;
  Complex impedance = {**mSeriesRes, omega * **mSeriesInd};
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

  // Create series rl sub component
  mSubSeriesElement = std::make_shared<SP::Ph1::ResIndSeries>(
      **mName + "_ResIndSeries", mLogLevel);
  mSubSeriesElement->connect({mTerminals[0]->node(), mTerminals[1]->node()});
  mSubSeriesElement->setParameters(**mSeriesRes, **mSeriesInd);
  mSubSeriesElement->initialize(mFrequencies);
  mSubSeriesElement->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubSeriesElement,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Create parallel sub components
  if (**mParallelCond >= 0) {
    mSubParallelResistor0 =
        std::make_shared<SP::Ph1::Resistor>(**mName + "_con0", mLogLevel);
    mSubParallelResistor0->setParameters(2. / **mParallelCond);
    mSubParallelResistor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubParallelResistor0->initialize(mFrequencies);
    mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelResistor0, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

    mSubParallelResistor1 =
        std::make_shared<SP::Ph1::Resistor>(**mName + "_con1", mLogLevel);
    mSubParallelResistor1->setParameters(2. / **mParallelCond);
    mSubParallelResistor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    mSubParallelResistor1->initialize(mFrequencies);
    mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelResistor1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  }

  if (**mParallelCap >= 0) {
    mSubParallelCapacitor0 =
        std::make_shared<SP::Ph1::Capacitor>(**mName + "_cap0", mLogLevel);
    mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubParallelCapacitor0->initialize(mFrequencies);
    mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor0, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubParallelCapacitor1 =
        std::make_shared<SP::Ph1::Capacitor>(**mName + "_cap1", mLogLevel);
    mSubParallelCapacitor1->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    mSubParallelCapacitor1->initialize(mFrequencies);
    mSubParallelCapacitor1->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

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

void SP::Ph1::PiLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::PiLine::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  this->mnaUpdateVoltage(**leftVector);
  this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::PiLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mSubSeriesElement->intfCurrent()(0, 0);
}

MNAInterface::List SP::Ph1::PiLine::mnaTearGroundComponents() {
  MNAInterface::List gndComponents;

  gndComponents.push_back(mSubParallelResistor0);
  gndComponents.push_back(mSubParallelResistor1);

  if (**mParallelCap >= 0) {
    gndComponents.push_back(mSubParallelCapacitor0);
    gndComponents.push_back(mSubParallelCapacitor1);
  }

  return gndComponents;
}

void SP::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
  mSubSeriesElement->mnaTearSetIdx(mTearIdx);
  mSubSeriesElement->mnaTearInitialize(omega, timeStep);
}

void SP::Ph1::PiLine::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  mSubSeriesElement->mnaTearApplyMatrixStamp(tearMatrix);
}

void SP::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix &voltageVector) {
  mSubSeriesElement->mnaTearApplyVoltageStamp(voltageVector);
}

void SP::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
  mSubSeriesElement->mnaTearPostStep(voltage - current * **mSeriesRes, current);
}
