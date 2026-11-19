// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph1_Capacitor.h>
#include <dpsim-models/Base/Base_Ph1_Inductor.h>
#include <dpsim-models/Base/Base_Ph1_Resistor.h>
#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
namespace SSN {
/// \brief Full_Serial_RLC
///
/// This element represents an one port circuit consisting of a resistor,
/// an inductor and a capacitor connected in series. The terminals are at
/// the beginning and the end of the component chain.
///	The states are the capacitor voltage and the inductor current, the output
/// is the latter of those states (inductor current). The input is the voltage
/// across the whole circuit. States and past inputs are updated after each
/// time step and are used to calculate the current (input) voltage,
/// represented as MNA node voltages.
/// SSN theory and variable naming based on
/// C. Dufour, J. Mahseredjian and J. Belanger,
/// "A combined state-space nodal method for the simulation of power system
/// transients," 2011 IEEE Power and Energy Society General Meeting, Detroit,
/// MI, USA, 2011, pp. 1-1, doi: 10.1109/PES.2011.6038887. keywords:
/// {Mathematical model;Analytical models;Equations;Power system transients;
/// Couplings;Switching circuits}

class Full_Serial_RLC final : public VTypeSSNComp,
                              public SharedFactory<Full_Serial_RLC>,
                              public Base::Ph1::Resistor,
                              public Base::Ph1::Inductor,
                              public Base::Ph1::Capacitor {
public:
  /// Defines UID, name, component parameters and logging level
  Full_Serial_RLC(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
      : Full_Serial_RLC(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;
  void setParameters(Real resistance, Real inductance, Real capacitance);

  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  Bool getRightVectorStampIndices(std::vector<UInt> &indices) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

private:
  MatrixComp buildInitialInputFromNodes(Real frequency) override;
};
} // namespace SSN
} // namespace Ph1
} // namespace EMT
} // namespace CPS
