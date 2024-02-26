/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/TopologicalNode.h>

namespace CPS {

class TopologicalTerminal : public IdentifiedObject {
public:
  typedef std::shared_ptr<TopologicalTerminal> Ptr;
  typedef std::vector<Ptr> List;
  /// Determines the connection between Component and Node
  PhaseType mPhaseType;
  /// Power through the Terminal
  MatrixComp mPower;
  ///
  TopologicalTerminal(String uid, String name,
                      PhaseType phase = PhaseType::Single);
  ///
  virtual ~TopologicalTerminal() {}
  /// Returns reference to TopologicalNode
  virtual TopologicalNode::Ptr topologicalNodes() = 0;
  /// Returns Power as complex matrix, where the size depends on the number of phases
  MatrixComp power() const;
  /// Returns single complex number for power
  Complex singlePower();
  ///
  void setPower(Complex power);
  ///
  void setPower(MatrixComp power);
  ///
  void setPhaseType(PhaseType type);
  ///
  Real singleActivePower();
  ///
  Real singleReactivePower();
  ///
  Complex initialSingleVoltage();
  ///
  MatrixComp initialVoltage();
  ///
  UInt matrixNodeIndex();
  ///
  std::vector<UInt> matrixNodeIndices();
};
} // namespace CPS
