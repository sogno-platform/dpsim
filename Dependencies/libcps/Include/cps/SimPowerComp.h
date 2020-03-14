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

#pragma once

#include <cps/TopologicalPowerComp.h>
#include <cps/SimTerminal.h>
#include <cps/SimNode.h>

namespace CPS {
	/// Base class for all components that are transmitting power.
	template <typename VarType>
	class SimPowerComp : public TopologicalPowerComp {

	protected:
		/// List of Terminals
		typename SimTerminal<VarType>::List mTerminals;
		/// List of virtual nodes
		typename SimNode<VarType>::List mVirtualNodes;
		/// Voltage between terminals
		MatrixVar<VarType> mIntfVoltage;
		/// Current through component
		MatrixVar<VarType> mIntfCurrent;
		/// List of considered network frequencies
		Matrix mFrequencies;
		/// Number of network frequencies
		UInt mNumFreqs = 0;
		///
		PhaseType mPhaseType = PhaseType::Single;

		/// "Cached" list of simulation nodes (to avoid shared_ptr accesses during simulation)
		std::vector<UInt> mMatrixNodeIndices;
		/// "Cached" flags for whether the connected nodes are grounded
		std::vector<bool> mMatrixNodeIndexIsGround;

		/// Flag indicating that parameters are set via setParameters() function
		bool parametersSet = false;

	public:
		typedef VarType Type;
		typedef std::shared_ptr<SimPowerComp<VarType>> Ptr;
		typedef std::vector<Ptr> List;

		/// Basic constructor that takes UID, name and log level
		SimPowerComp(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Basic constructor that takes name and log level and sets the UID to name as well
		SimPowerComp(String name, Logger::Level logLevel = Logger::Level::off)
			: SimPowerComp(name, name, logLevel) { }
		/// Destructor - does not do anything
		virtual ~SimPowerComp() { }

		/// Returns a modified copy of the component with the given suffix added to the name and without
		/// connected nodes / terminals
		/// TODO should be abstract and implemented everywhere
		virtual Ptr clone(String name) { return nullptr; }

		// #### Terminals ####
		/// Returns nominal number of Terminals for this component type.
		UInt terminalNumber() { return mNumTerminals; }
		/// Returns the number of connected Terminals
		UInt terminalNumberConnected();
		///
		Bool hasUnconnectedTerminals();
		///
		void checkForUnconnectedTerminals();
		/// Return list of Terminal pointers
		typename SimTerminal<VarType>::List terminals() { return mTerminals; }
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
		void setTerminalAt(typename SimTerminal<VarType>::Ptr terminal, UInt terminalPosition);

		/// Update the "cached" mMatrixNodeIndices and mMatrixNodeIndexIsGround members
		void updateMatrixNodeIndices();

		// #### Nodes ####
		/// Returns the actual number of Nodes / Terminals that are already set to valid Nodes.
		UInt nodeNumber();
		/// Get pointer to node
		typename SimNode<VarType>::Ptr node(UInt index) {
			if (index >= mTerminals.size()) {
				throw SystemError("Node not available for " + mUID);
			}
			return mTerminals[index]->node();
		};
		UInt matrixNodeIndex(UInt nodeIndex) {
			return mMatrixNodeIndices[nodeIndex * 3];
		}
		UInt matrixNodeIndex(UInt nodeIndex, UInt phaseIndex) {
			return mMatrixNodeIndices[nodeIndex * 3 + phaseIndex];
		}
		/// TODO replace with access to mMatrixNodeIndices
		std::vector<UInt> matrixNodeIndices(UInt index) { return node(index)->matrixNodeIndices(); }
		/// Get nodes as base type TopologicalNode
		TopologicalNode::List topologicalNodes();

		// #### Virtual Nodes ####
		/// Returns nominal number of virtual nodes for this component type.
		UInt virtualNodesNumber() { return mNumVirtualNodes; }
		/// Returns true if virtual node number is greater than zero.
		Bool hasVirtualNodes() { return mNumVirtualNodes > 0; }
		///
		typename SimNode<VarType>::List& virtualNodes() { return mVirtualNodes; }
		/// Get pointer to virtual node
		typename SimNode<VarType>::Ptr virtualNode(UInt index);
		/// Get vector of simulation node numbers from virtual Node
		std::vector<UInt> virtualMatrixNodeIndices(UInt index) { return virtualNode(index)->matrixNodeIndices(); }
		/// Get simulation node number from virtual node
		UInt virtualSimNode(UInt nodeIndex, UInt phaseIndex = 0) { return virtualMatrixNodeIndices(nodeIndex)[phaseIndex]; }

		// #### States ####
		const MatrixVar<VarType>& intfCurrent() { return mIntfCurrent; }
		///
		const MatrixVar<VarType>& intfVoltage() { return mIntfVoltage; }
		///
		MatrixComp initialVoltage(UInt index) { return mTerminals[index]->initialVoltage(); }
		///
		Complex initialSingleVoltage(UInt index) { return mTerminals[index]->initialSingleVoltage(); }
		///
		Bool terminalNotGrounded(UInt index) { return !mMatrixNodeIndexIsGround[index]; }

		// #### Setters ####
		void setIntfCurrent(MatrixVar<VarType> current) { mIntfCurrent = current; }
		///
		void setIntfVoltage(MatrixVar<VarType> voltage) { mIntfVoltage = voltage; }
		///
		void setVirtualNodeNumber(UInt num);
		/// Sets the virtual node at index nodeNum.
		void setVirtualNodeAt(typename SimNode<VarType>::Ptr virtualNode, UInt nodeNum);
		/// Sets all nodes and checks for nominal number of Nodes for this Component.
		void connect(typename SimNode<VarType>::List nodes);

		// #### Calculations ####
		/// Initialize components with correct network frequencies
		virtual void initialize(Matrix frequencies);
		/// Initializes Component variables according to power flow data stored in Nodes.
		virtual void initializeFromPowerflow(Real frequency) { }
	};
}
