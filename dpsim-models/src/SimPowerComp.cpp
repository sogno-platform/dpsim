/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SimPowerComp.h>

using namespace CPS;

template <typename VarType>
SimPowerComp<VarType>::SimPowerComp(String uid, String name,
                                    Logger::Level logLevel)
    : TopologicalPowerComp(uid, name, logLevel),
      mIntfVoltage(mAttributes->create<MatrixVar<VarType>>("v_intf")),
      mIntfCurrent(mAttributes->create<MatrixVar<VarType>>("i_intf")) {
  mTerminals.clear();
}

template <typename VarType>
typename SimPowerComp<VarType>::Ptr SimPowerComp<VarType>::clone(String name) {
  return nullptr;
}

template <typename VarType> UInt SimPowerComp<VarType>::terminalNumber() {
  return mNumTerminals;
}

template <typename VarType>
typename SimTerminal<VarType>::List SimPowerComp<VarType>::terminals() {
  return mTerminals;
}

template <typename VarType>
typename SimNode<VarType>::Ptr SimPowerComp<VarType>::node(UInt index) {
  if (index >= mTerminals.size()) {
    throw SystemError("Node not available for " + **mUID);
  }
  return mTerminals[index]->node();
};

template <typename VarType>
UInt SimPowerComp<VarType>::matrixNodeIndex(UInt nodeIndex) {
  return mMatrixNodeIndices[nodeIndex * 3];
}

template <typename VarType>
UInt SimPowerComp<VarType>::matrixNodeIndex(UInt nodeIndex, UInt phaseIndex) {
  return mMatrixNodeIndices[nodeIndex * 3 + phaseIndex];
}

template <typename VarType>
std::vector<UInt> SimPowerComp<VarType>::matrixNodeIndices(UInt index) {
  return node(index)->matrixNodeIndices();
}

template <typename VarType> UInt SimPowerComp<VarType>::virtualNodesNumber() {
  return mNumVirtualNodes;
}

template <typename VarType> Bool SimPowerComp<VarType>::hasVirtualNodes() {
  return mNumVirtualNodes > 0;
}

template <typename VarType> Bool SimPowerComp<VarType>::hasSubComponents() {
  return mSubComponents.size() > 0;
}

template <typename VarType>
typename SimPowerComp<VarType>::List SimPowerComp<VarType>::subComponents() {
  return mSubComponents;
}

template <typename VarType>
typename SimNode<VarType>::List &SimPowerComp<VarType>::virtualNodes() {
  return mVirtualNodes;
}

template <typename VarType>
std::vector<UInt> SimPowerComp<VarType>::virtualMatrixNodeIndices(UInt index) {
  return virtualNode(index)->matrixNodeIndices();
}

template <typename VarType>
UInt SimPowerComp<VarType>::virtualSimNode(UInt nodeIndex, UInt phaseIndex) {
  return virtualMatrixNodeIndices(nodeIndex)[phaseIndex];
}

template <typename VarType>
const MatrixVar<VarType> &SimPowerComp<VarType>::intfCurrent() {
  return **mIntfCurrent;
}

template <typename VarType>
const MatrixVar<VarType> &SimPowerComp<VarType>::intfVoltage() {
  return **mIntfVoltage;
}

template <typename VarType>
MatrixComp SimPowerComp<VarType>::initialVoltage(UInt index) {
  return mTerminals[index]->initialVoltage();
}

template <typename VarType>
Complex SimPowerComp<VarType>::initialSingleVoltage(UInt index) {
  return mTerminals[index]->initialSingleVoltage();
}

template <typename VarType>
Bool SimPowerComp<VarType>::terminalNotGrounded(UInt index) {
  return !mMatrixNodeIndexIsGround[index];
}

template <typename VarType>
void SimPowerComp<VarType>::setIntfCurrent(MatrixVar<VarType> current) {
  **mIntfCurrent = current;
}

template <typename VarType>
void SimPowerComp<VarType>::setIntfVoltage(MatrixVar<VarType> voltage) {
  **mIntfVoltage = voltage;
}

// #### Terminals ####
template <typename VarType>
UInt SimPowerComp<VarType>::terminalNumberConnected() {
  return (UInt)(mNumTerminals -
                std::count(mTerminals.begin(), mTerminals.end(), nullptr));
}

template <typename VarType>
Bool SimPowerComp<VarType>::hasUnconnectedTerminals() {
  return (std::count(mTerminals.begin(), mTerminals.end(), nullptr) > 0);
}

template <typename VarType>
void SimPowerComp<VarType>::checkForUnconnectedTerminals() {
  if (hasUnconnectedTerminals()) {
    throw SystemError("Found unconnected Terminals for " + **mUID);
  }
}

template <typename VarType>
typename SimTerminal<VarType>::Ptr SimPowerComp<VarType>::terminal(UInt index) {
  if (index >= mTerminals.size()) {
    throw SystemError("Terminal not available for " + **mUID);
  }
  return mTerminals[index];
}

template <typename VarType>
TopologicalTerminal::List SimPowerComp<VarType>::topologicalTerminals() {
  TopologicalTerminal::List terminals;
  for (typename SimTerminal<VarType>::Ptr term : mTerminals) {
    terminals.push_back(term);
  }
  return terminals;
}

template <typename VarType>
void SimPowerComp<VarType>::setTerminalNumber(UInt num) {
  mNumTerminals = num;
  mTerminals.resize(mNumTerminals, nullptr);
  mMatrixNodeIndices.resize(3 * num);
  mMatrixNodeIndexIsGround.resize(num);
}

template <typename VarType>
void SimPowerComp<VarType>::setTerminals(
    typename SimTerminal<VarType>::List terminals) {
  if (mNumTerminals < terminals.size()) {
    SPDLOG_LOGGER_ERROR(
        mSLog, "Number of Terminals is too large for Component {} - Ignoring",
        **mName);
    return;
  }
  mTerminals = terminals;
}

template <typename VarType>
void SimPowerComp<VarType>::setTerminalAt(
    typename SimTerminal<VarType>::Ptr terminal, UInt terminalPosition) {
  if (mNumTerminals <= terminalPosition) {
    SPDLOG_LOGGER_ERROR(
        mSLog, "Terminal position number too large for Component {} - Ignoring",
        **mName);
    return;
  }
  mTerminals[terminalPosition] = terminal;
  SPDLOG_LOGGER_INFO(
      mSLog, "Set Terminal at position {} to Node {}, simulation node {}",
      terminalPosition, mTerminals[terminalPosition]->node()->name(),
      mTerminals[terminalPosition]->matrixNodeIndex());
}

template <typename VarType>
void SimPowerComp<VarType>::updateMatrixNodeIndices() {
  for (UInt nodeIdx = 0; nodeIdx < mNumTerminals; nodeIdx++) {
    mMatrixNodeIndices[3 * nodeIdx] =
        node(nodeIdx)->matrixNodeIndex(PhaseType::A);
    mMatrixNodeIndices[3 * nodeIdx + 1] =
        node(nodeIdx)->matrixNodeIndex(PhaseType::B);
    mMatrixNodeIndices[3 * nodeIdx + 2] =
        node(nodeIdx)->matrixNodeIndex(PhaseType::C);
    mMatrixNodeIndexIsGround[nodeIdx] = node(nodeIdx)->isGround();
  }
}

// #### Nodes ####
template <typename VarType> UInt SimPowerComp<VarType>::nodeNumber() {
  return static_cast<UInt>(
      std::count(mTerminals.begin(), mTerminals.end(), nullptr));
}

template <typename VarType>
TopologicalNode::List SimPowerComp<VarType>::topologicalNodes() {
  TopologicalNode::List nodes;
  for (typename SimTerminal<VarType>::Ptr term : mTerminals) {
    nodes.push_back(term->node());
  }
  return nodes;
}

// #### Virtual Nodes ####
template <typename VarType>
void SimPowerComp<VarType>::setVirtualNodeNumber(UInt num) {
  mNumVirtualNodes = num;
  mVirtualNodes.resize(mNumVirtualNodes, nullptr);

  for (UInt idx = 0; idx < mNumVirtualNodes; idx++) {
    String nodeName = **mName + "_vnode_" + std::to_string(idx);
    setVirtualNodeAt(std::make_shared<SimNode<VarType>>(nodeName, mPhaseType),
                     idx);
  }
}

template <typename VarType>
void SimPowerComp<VarType>::setVirtualNodeAt(
    typename SimNode<VarType>::Ptr virtualNode, UInt nodeNum) {
  if (mNumVirtualNodes <= nodeNum) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "Virtual Node position number too large for Component {} - Ignoring",
        **mName);
  }
  mVirtualNodes[nodeNum] = virtualNode;
  SPDLOG_LOGGER_INFO(
      mSLog, "Set virtual Node at position {} to Node {}, simulation node {}",
      nodeNum, mVirtualNodes[nodeNum]->name(),
      mVirtualNodes[nodeNum]->matrixNodeIndex());
}

template <typename VarType>
typename SimNode<VarType>::Ptr SimPowerComp<VarType>::virtualNode(UInt index) {
  if (index >= mVirtualNodes.size()) {
    throw SystemError("Node not available for " + **mUID);
  }
  return mVirtualNodes[index];
}

// #### Other functions ####
template <typename VarType>
void SimPowerComp<VarType>::connect(typename SimNode<VarType>::List nodes) {
  if (mNumTerminals < nodes.size()) {
    SPDLOG_LOGGER_ERROR(
        mSLog, "Number of Nodes is too large for Component {} - Ignoring",
        **mName);
    return;
  }
  for (UInt i = 0; i < nodes.size(); i++) {
    String name = **mName + "_T" + std::to_string(i);
    typename SimTerminal<VarType>::Ptr terminal =
        SimTerminal<VarType>::make(name);
    terminal->setNode(nodes[i]);
    setTerminalAt(terminal, i);
  }
}

template <typename VarType>
void SimPowerComp<VarType>::initialize(Matrix frequencies) {
  mFrequencies = frequencies;
  mNumFreqs = static_cast<UInt>(mFrequencies.size());

  if (mPhaseType != PhaseType::ABC) {
    **mIntfVoltage = MatrixVar<VarType>::Zero(1, mNumFreqs);
    **mIntfCurrent = MatrixVar<VarType>::Zero(1, mNumFreqs);
  } else {
    **mIntfVoltage = MatrixVar<VarType>::Zero(3, mNumFreqs);
    **mIntfCurrent = MatrixVar<VarType>::Zero(3, mNumFreqs);
  }

  for (auto node : mVirtualNodes)
    node->initialize(frequencies);
}

// Declare specializations to move definitions to .cpp
template class CPS::SimPowerComp<Real>;
template class CPS::SimPowerComp<Complex>;
