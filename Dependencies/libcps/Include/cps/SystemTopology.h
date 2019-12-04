/** Power System Topology
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <vector>
#include <algorithm>

#include <cps/PowerComponent.h>
#include <cps/TopologicalComponent.h>
#include <cps/PowerComponent.h>
#include <cps/Node.h>

#ifdef WITH_GRAPHVIZ
  #include <cps/Graph.h>
#endif

namespace CPS {
	class SystemTopology {
	public:
		using Ptr = std::shared_ptr<SystemTopology>;

		/// List of considered network frequencies
		Matrix mFrequencies;
		/// List of network nodes
		TopologicalNode::List mNodes;
		/// List of network components
		Component::List mComponents;
		/// List of tearing components could be used
		/// by a solver to split the network into subnetworks
		Component::List mTearComponents;
		/// Map of network components connected to network nodes
		std::map<TopologicalNode::Ptr, Component::List> mComponentsAtNode;

		// #### Deprecated ####
		// Better use mFrequencies
		/// System frequency
		Real mSystemFrequency;
		/// System angular frequency - omega
		Real mSystemOmega;

		/// Do not use this constructor
		SystemTopology() { }

		/// Constructor to be used if components
		/// and nodes are added one by one

		SystemTopology(Real frequency) {
			mSystemFrequency = frequency;
			mSystemOmega = 2 * PI * mSystemFrequency;

			Matrix frequencies(1,1);
			frequencies << mSystemFrequency;
			mFrequencies = frequencies;
		}

		/// This constructor requires a search for all nodes
		/// which is not implemented yet!
		SystemTopology(Real frequency, Component::List components)
		: SystemTopology(frequency) {
			mComponents = components;
		}

		/// Standard constructor for single frequency simulations
		SystemTopology(Real frequency, TopologicalNode::List nodes, Component::List components)
		: SystemTopology(frequency) {
			addNodes(nodes);
			addComponents(components);
		}

		/// Standard constructor for multi frequency simulations
		SystemTopology(Real frequency, Matrix frequencies, TopologicalNode::List nodes, Component::List components)
		: SystemTopology(frequency) {
			mFrequencies = frequencies;
			addNodes(nodes);
			addComponents(components);
		}

		/// Reset state of components
		void reset();

		// #### Add Objects to SystemTopology ####

		/// Adds node and initializes frequencies
		void addNode(TopologicalNode::Ptr topNode) {
			auto nodeComplex = std::dynamic_pointer_cast<Node<Complex>>(topNode);
			if (nodeComplex) nodeComplex->initialize(mFrequencies);

			auto nodeReal = std::dynamic_pointer_cast<Node<Real>>(topNode);
			if (nodeReal) nodeReal->initialize(mFrequencies);

			mNodes.push_back(topNode);
		}

		/// Adds node at specified position and initializes frequencies
		void addNodeAt(TopologicalNode::Ptr topNode, UInt index) {
			auto node = std::dynamic_pointer_cast<Node<Complex>>(topNode);
			if (node) node->initialize(mFrequencies);

			auto nodeReal = std::dynamic_pointer_cast<Node<Real>>(topNode);
			if (nodeReal) nodeReal->initialize(mFrequencies);

			if (index > mNodes.capacity())
				mNodes.resize(index+1);

			mNodes[index] = topNode;
		}

		/// Add multiple nodes
		void addNodes(TopologicalNode::List topNodes) {
			for (auto topNode : topNodes)
				addNode(topNode);
		}

		/// Adds component and initializes frequencies
		void addComponent(Component::Ptr component) {
			auto powerCompComplex = std::dynamic_pointer_cast<PowerComponent<Complex>>(component);
			if (powerCompComplex) powerCompComplex->initialize(mFrequencies);

			auto powerCompReal = std::dynamic_pointer_cast<PowerComponent<Real>>(component);
			if (powerCompReal) powerCompReal->initialize(mFrequencies);

			mComponents.push_back(component);
		}

		/// Add multiple components
		void addComponents(Component::List components) {
			for (auto comp : components)
				addComponent(comp);
		}

		/// Adds component and initializes frequencies
		void addTearComponent(Component::Ptr component) {
			auto powerCompComplex = std::dynamic_pointer_cast<PowerComponent<Complex>>(component);
			if (powerCompComplex) powerCompComplex->initialize(mFrequencies);

			auto powerCompReal = std::dynamic_pointer_cast<PowerComponent<Real>>(component);
			if (powerCompReal) powerCompReal->initialize(mFrequencies);

			mTearComponents.push_back(component);
		}

		/// Add multiple components
		void addTearComponents(Component::List components) {
			for (auto comp : components)
				addTearComponent(comp);
		}

		// #### Get Objects from SystemTopology ####

		/// Returns TopologicalNode by index in node list
		template<typename Type>
		typename std::shared_ptr<Type> node(UInt index) {
			if (index < mNodes.size()) {
				auto topoNode = mNodes[index];
				auto node = std::dynamic_pointer_cast<Type>(topoNode);
				if (node)
					return node;
			}

			return nullptr;
		}

		/// Returns TopologicalNode by name
		template<typename Type>
		typename std::shared_ptr<Type> node(const String &name) {
			for (auto topoNode : mNodes) {
				if (topoNode->name() == name) {
					auto node = std::dynamic_pointer_cast<Type>(topoNode);
					if (node)
						return node;
					else
						return nullptr;
				}
			}
			return nullptr;
		}

		/// Returns Component by name
		template<typename Type>
		typename std::shared_ptr<Type> component(const String &name) {
			for (auto comp : mComponents) {
				if (comp->name() == name) {
					auto comp2 = std::dynamic_pointer_cast<Type>(comp);
					if (comp2)
						return comp2;
					else
						return nullptr;
				}
			}
			return nullptr;
		}

		// #### Operations on the SystemTopology ####

		/// Copy the whole topology the given number of times and add the resulting components and nodes to the topology.
		void multiply(Int numberCopies);

		///
		template <typename VarType>
		int checkTopologySubnets(std::unordered_map<typename CPS::Node<VarType>::Ptr, int>& subnet);

		///
		template <typename VarType>
		void splitSubnets(std::vector<CPS::SystemTopology>& splitSystems);

#ifdef WITH_GRAPHVIZ
		Graph::Graph topologyGraph();
		void printGraph(Graph::Graph graph);
#endif

	private:
		template<typename VarType>
		void multiplyPowerComps(Int numberCopies);
	};
}
