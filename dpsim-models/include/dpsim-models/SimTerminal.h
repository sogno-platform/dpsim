/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimNode.h>
#include <dpsim-models/TopologicalTerminal.h>

namespace CPS {

template <typename VarType>
class SimTerminal : public TopologicalTerminal,
                    public SharedFactory<SimTerminal<VarType>> {
protected:
  MatrixVar<VarType> mCurrent;
  std::weak_ptr<SimNode<VarType>> mNode;

public:
  typedef std::shared_ptr<SimTerminal<VarType>> Ptr;
  typedef std::vector<Ptr> List;
  ///
  SimTerminal(String name) : TopologicalTerminal(name, name) {}
  ///
  SimTerminal(String uid, String name) : TopologicalTerminal(uid, name) {}
  ///
  typename SimNode<VarType>::Ptr node() { return mNode.lock(); }
  ///
  void setNode(typename SimNode<VarType>::Ptr node) {
    mNode = node;
    setPhaseType(node->phaseType());
  }
  ///
  TopologicalNode::Ptr topologicalNodes() { return node(); }
  ///
  VarType singleVoltage() {
    if (node()->isGround())
      return 0.;
    else
      return node()->singleVoltage(mPhaseType);
  }
  ///
  MatrixVar<VarType> voltage() {
    if (node()->isGround())
      return {0., 0., 0.};
    else
      return node()->voltage();
  }
};
} // namespace CPS
