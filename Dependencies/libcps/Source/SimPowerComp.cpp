/**
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/SimPowerComp.h>

using namespace CPS;

template <typename VarType>
SimPowerComp<VarType>::SimPowerComp(String uid, String name, Logger::Level logLevel)
	: TopologicalPowerComp(uid, name, logLevel) {
	addAttribute<MatrixVar<VarType>>("v_intf", &mIntfVoltage, Flags::read);
	addAttribute<MatrixVar<VarType>>("i_intf", &mIntfCurrent, Flags::read);
	mTerminals.clear();
}

// #### Terminals ####
template <typename VarType>
UInt SimPowerComp<VarType>::terminalNumberConnected() {
	return (UInt)(mNumTerminals - std::count(mTerminals.begin(), mTerminals.end(), nullptr));
}

template <typename VarType>
Bool SimPowerComp<VarType>::hasUnconnectedTerminals() {
	return (std::count(mTerminals.begin(), mTerminals.end(), nullptr) > 0);
}

template <typename VarType>
void SimPowerComp<VarType>::checkForUnconnectedTerminals() {
	if ( hasUnconnectedTerminals() ) {
		throw SystemError("Found unconnected Terminals for " + mUID);
	}
}

template <typename VarType>
typename Terminal<VarType>::Ptr SimPowerComp<VarType>::terminal(UInt index) {
	if (index >= mTerminals.size()) {
		throw SystemError("Terminal not available for " + mUID);
	}
	return mTerminals[index];
}

template <typename VarType>
TopologicalTerminal::List SimPowerComp<VarType>::topologicalTerminals() {
	TopologicalTerminal::List terminals;
	for (typename Terminal<VarType>::Ptr term : mTerminals) {
		terminals.push_back(term);
	}
	return terminals;
}

template <typename VarType>
void SimPowerComp<VarType>::setTerminalNumber(UInt num) {
	mNumTerminals = num;
	mTerminals.resize(mNumTerminals, nullptr);
	mMatrixNodeIndices.resize(3*num);
	mMatrixNodeIndexIsGround.resize(num);
}

template <typename VarType>
void SimPowerComp<VarType>::setTerminals(typename Terminal<VarType>::List terminals) {
	if (mNumTerminals < terminals.size()) {
		mSLog->error("Number of Terminals is too large for Component {} - Ignoring", mName);
		return;
	}
	mTerminals = terminals;
}

template <typename VarType>
void SimPowerComp<VarType>::setTerminalAt(typename Terminal<VarType>::Ptr terminal, UInt terminalPosition) {
	if (mNumTerminals <= terminalPosition) {
		mSLog->error("Terminal position number too large for Component {} - Ignoring", mName);
		return;
	}
	mTerminals[terminalPosition] = terminal;
	mSLog->info("Set Terminal at position {} to Node {}, simulation node {}", terminalPosition, mTerminals[terminalPosition]->node()->name(), mTerminals[terminalPosition]->matrixNodeIndex());
}

template <typename VarType>
void SimPowerComp<VarType>::updateMatrixNodeIndices() {
	for (UInt nodeIdx = 0; nodeIdx < mNumTerminals; nodeIdx++) {
		mMatrixNodeIndices[3*nodeIdx] = node(nodeIdx)->matrixNodeIndex(PhaseType::A);
		mMatrixNodeIndices[3*nodeIdx+1] = node(nodeIdx)->matrixNodeIndex(PhaseType::B);
		mMatrixNodeIndices[3*nodeIdx+2] = node(nodeIdx)->matrixNodeIndex(PhaseType::C);
		mMatrixNodeIndexIsGround[nodeIdx] = node(nodeIdx)->isGround();
	}
}

// #### Nodes ####
template <typename VarType>
UInt SimPowerComp<VarType>::nodeNumber() {
	return static_cast<UInt>(std::count(mTerminals.begin(), mTerminals.end(), nullptr));
}

template <typename VarType>
TopologicalNode::List SimPowerComp<VarType>::topologicalNodes() {
	TopologicalNode::List nodes;
	for (typename Terminal<VarType>::Ptr term : mTerminals) {
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
		String nodeName = mName + "_vnode_" + std::to_string(idx);
		setVirtualNodeAt(std::make_shared<SimNode<VarType>>(nodeName, mPhaseType), idx);
	}
}

template <typename VarType>
void SimPowerComp<VarType>::setVirtualNodeAt(typename SimNode<VarType>::Ptr virtualNode, UInt nodeNum) {
	if (mNumVirtualNodes <= nodeNum) {
		mSLog->error("Virtual Node position number too large for Component {} - Ignoring", mName);
	}
	mVirtualNodes[nodeNum] = virtualNode;
	mSLog->info("Set virtual Node at position {} to Node {}, simulation node {}",
		nodeNum, mVirtualNodes[nodeNum]->name(), mVirtualNodes[nodeNum]->matrixNodeIndex());
}

template <typename VarType>
typename SimNode<VarType>::Ptr SimPowerComp<VarType>::virtualNode(UInt index) {
	if (index >= mVirtualNodes.size()) {
		throw SystemError("Node not available for " + mUID);
	}
	return mVirtualNodes[index];
}

// #### Other functions ####
template<typename VarType>
void SimPowerComp<VarType>::connect(typename SimNode<VarType>::List nodes) {
	if (mNumTerminals < nodes.size()) {
		mSLog->error("Number of Nodes is too large for Component {} - Ignoring", mName);
		return;
	}
	for (UInt i = 0; i < nodes.size(); i++) {
		String name = mName + "_T" + std::to_string(i);
		typename Terminal<VarType>::Ptr terminal = Terminal<VarType>::make(name);
		terminal->setNode(nodes[i]);
		setTerminalAt(terminal, i);
	}
}

template<typename VarType>
void SimPowerComp<VarType>::initialize(Matrix frequencies) {
	mFrequencies = frequencies;
	mNumFreqs = static_cast<UInt>(mFrequencies.size());

	if (mPhaseType != PhaseType::ABC) {
		mIntfVoltage = MatrixVar<VarType>::Zero(1, mNumFreqs);
		mIntfCurrent = MatrixVar<VarType>::Zero(1, mNumFreqs);
	}
	else {
		mIntfVoltage = MatrixVar<VarType>::Zero(3, mNumFreqs);
		mIntfCurrent = MatrixVar<VarType>::Zero(3, mNumFreqs);
	}

	for (auto node : mVirtualNodes)
		node->initialize(frequencies);
}

// Declare specializations to move definitions to .cpp
template class CPS::SimPowerComp<Real>;
template class CPS::SimPowerComp<Complex>;
