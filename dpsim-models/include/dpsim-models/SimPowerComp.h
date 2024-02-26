/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimNode.h>
#include <dpsim-models/SimTerminal.h>
#include <dpsim-models/TopologicalPowerComp.h>

namespace CPS {
/// Base class for all components that are transmitting power.
template <typename VarType> class SimPowerComp : public TopologicalPowerComp {

protected:
  /// List of Terminals
  typename SimTerminal<VarType>::List mTerminals;
  /// List of virtual nodes
  typename SimNode<VarType>::List mVirtualNodes;
  /// List of considered network frequencies
  Matrix mFrequencies;
  /// Number of network frequencies
  UInt mNumFreqs = 0;
  ///
  PhaseType mPhaseType = PhaseType::Single;

  /// List of subcomponents
  /// DEPRECATED: Delete or move into CompositePowerComp
  typename std::vector<std::shared_ptr<SimPowerComp<VarType>>> mSubComponents;
  /// "Cached" list of simulation nodes (to avoid shared_ptr accesses during simulation)
  std::vector<UInt> mMatrixNodeIndices;
  /// "Cached" flags for whether the connected nodes are grounded
  std::vector<bool> mMatrixNodeIndexIsGround;

public:
  typedef VarType Type;
  typedef std::shared_ptr<SimPowerComp<VarType>> Ptr;
  typedef std::vector<Ptr> List;

  /// Voltage between terminals
  const typename Attribute<MatrixVar<VarType>>::Ptr mIntfVoltage;
  /// Current through component
  const typename Attribute<MatrixVar<VarType>>::Ptr mIntfCurrent;

  /// Basic constructor that takes UID, name and log level
  SimPowerComp(String uid, String name,
               Logger::Level logLevel = Logger::Level::off);
  /// Basic constructor that takes name and log level and sets the UID to name as well
  SimPowerComp(String name, Logger::Level logLevel = Logger::Level::off)
      : SimPowerComp(name, name, logLevel) {}
  /// Destructor - does not do anything
  virtual ~SimPowerComp() {}

  /// Returns a modified copy of the component with the given suffix added to the name and without
  /// connected nodes / terminals
  /// DEPRECATED: This method should be removed
  virtual Ptr clone(String name);

  // #### Terminals ####
  /// Returns nominal number of Terminals for this component type.
  UInt terminalNumber();
  /// Returns the number of connected Terminals
  UInt terminalNumberConnected();
  ///
  Bool hasUnconnectedTerminals();
  ///
  void checkForUnconnectedTerminals();
  /// Return list of Terminal pointers
  typename SimTerminal<VarType>::List terminals();
  /// Get pointer to Terminal
  typename SimTerminal<VarType>::Ptr terminal(UInt index);
  /// Returns the list of terminals as TopologicalTerminal pointers
  TopologicalTerminal::List topologicalTerminals();
  ///
  void setTerminalNumber(UInt num);
  /// Sets all Terminals of the component and checks if the number of Terminals is too large
  /// fir this component type.
  void setTerminals(typename SimTerminal<VarType>::List terminals);
  /// Sets Terminal at index terminalPosition.
  void setTerminalAt(typename SimTerminal<VarType>::Ptr terminal,
                     UInt terminalPosition);

  /// Update the "cached" mMatrixNodeIndices and mMatrixNodeIndexIsGround members
  void updateMatrixNodeIndices();

  // #### Nodes ####
  /// Returns the actual number of Nodes / Terminals that are already set to valid Nodes.
  UInt nodeNumber();
  /// Get pointer to node
  typename SimNode<VarType>::Ptr node(UInt index);

  UInt matrixNodeIndex(UInt nodeIndex);

  UInt matrixNodeIndex(UInt nodeIndex, UInt phaseIndex);
  /// TODO replace with access to mMatrixNodeIndices
  std::vector<UInt> matrixNodeIndices(UInt index);
  /// Get nodes as base type TopologicalNode
  TopologicalNode::List topologicalNodes();

  // #### Virtual Nodes ####
  /// Returns nominal number of virtual nodes for this component type.
  UInt virtualNodesNumber();
  /// Returns true if virtual node number is greater than zero.
  Bool hasVirtualNodes();
  /// Returns true if subcomponents included in this component
  Bool hasSubComponents();
  /// Get list of subcomponents
  typename SimPowerComp<VarType>::List subComponents();
  ///
  typename SimNode<VarType>::List &virtualNodes();
  /// Get pointer to virtual node
  typename SimNode<VarType>::Ptr virtualNode(UInt index);
  /// Get vector of simulation node numbers from virtual Node
  std::vector<UInt> virtualMatrixNodeIndices(UInt index);
  /// Get simulation node number from virtual node
  UInt virtualSimNode(UInt nodeIndex, UInt phaseIndex = 0);

  // #### States ####
  const MatrixVar<VarType> &intfCurrent();
  ///
  const MatrixVar<VarType> &intfVoltage();
  ///
  MatrixComp initialVoltage(UInt index);
  ///
  Complex initialSingleVoltage(UInt index);
  ///
  Bool terminalNotGrounded(UInt index);

  // #### Setters ####
  void setIntfCurrent(MatrixVar<VarType> current);
  ///
  void setIntfVoltage(MatrixVar<VarType> voltage);
  ///
  void setVirtualNodeNumber(UInt num);
  /// Sets the virtual node at index nodeNum.
  void setVirtualNodeAt(typename SimNode<VarType>::Ptr virtualNode,
                        UInt nodeNum);
  /// Sets all nodes and checks for nominal number of Nodes for this Component.
  void connect(typename SimNode<VarType>::List nodes);

  // #### Calculations ####
  /// Initialize components with correct network frequencies
  virtual void initialize(Matrix frequencies);
  /// Initializes Component variables according to power flow data stored in Nodes.
  virtual void initializeFromNodesAndTerminals(Real frequency) {}
};
} // namespace CPS
