// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <dpsim-models/Definitions.h>

#include <array>

namespace CPS::EMT {

/// Runtime-independent description of a linear three-phase SSN multiport.
///
/// The port admittance is the D part of the discrete Norton/ABCD model.  The
/// remaining matrices are the component-specific A/B realization used to
/// update the companion-model history exactly as in the original component.
/// The descriptor is populated once during initialization and is then owned
/// by LinearSSNFused; no child component is called during a simulation step.
struct LinearSSNModel {
  enum class Kind { PiLine, ImpedanceLoad, Transformer };

  Kind kind;
  UInt terminalCount = 0;
  std::array<UInt, 6> nodeIndices{};
  std::array<String, 6> nodeNames{};
  std::array<Bool, 2> grounded{{true, true}};
  Matrix portAdmittance;
  Matrix state;

  // Pi-line realization.
  Matrix seriesResistance;
  Matrix inductorAdmittance;
  Matrix seriesSystemInverse;
  Matrix seriesAdmittance;
  Matrix capAdmittance;
  Matrix parallelConductance;

  // Impedance-load realization.
  Real totalConductance = 0.;
  Real inductorAdmittanceScalar = 0.;
  Real capacitorAdmittanceScalar = 0.;
  Bool hasLoadInductance = false;
  Bool hasLoadCapacitance = false;

  // Transformer realization.
  Matrix seriesResistanceHalf;
  Matrix magnetizingInductorAdmittance;
  Matrix magnetizingConductance;
  Matrix midpointInverse;
  UInt referenceTerminal = 0;
  Real ratioFromReference = 1.;
  Real orientationSign = 1.;
};

} // namespace CPS::EMT
