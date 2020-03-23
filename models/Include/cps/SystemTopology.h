/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>
#include <algorithm>

#include <cps/TopologicalPowerComp.h>
#include <cps/SimPowerComp.h>
#include <cps/SimNode.h>

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
		IdentifiedObject::List mComponents;
		/// List of tearing components could be used
		/// by a solver to split the network into subnetworks
		IdentifiedObject::List mTearComponents;
		/// Map of network components connected to network nodes
		std::map<TopologicalNode::Ptr, TopologicalPowerComp::List> mComponentsAtNode;

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
		SystemTopology(Real frequency, IdentifiedObject::List components)
		: SystemTopology(frequency) {
			mComponents = components;
		}

		/// Standard constructor for single frequency simulations
		SystemTopology(Real frequency, TopologicalNode::List nodes, IdentifiedObject::List components)
		: SystemTopology(frequency) {
			addNodes(nodes);
			addComponents(components);
		}

		/// Standard constructor for multi frequency simulations
		SystemTopology(Real frequency, Matrix frequencies, TopologicalNode::List nodes, IdentifiedObject::List components)
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
			auto nodeComplex = std::dynamic_pointer_cast<SimNode<Complex>>(topNode);
			if (nodeComplex) nodeComplex->initialize(mFrequencies);

			auto nodeReal = std::dynamic_pointer_cast<SimNode<Real>>(topNode);
			if (nodeReal) nodeReal->initialize(mFrequencies);

			mNodes.push_back(topNode);
		}

		/// Adds node at specified position and initializes frequencies
		void addNodeAt(TopologicalNode::Ptr topNode, UInt index) {
			auto node = std::dynamic_pointer_cast<SimNode<Complex>>(topNode);
			if (node) node->initialize(mFrequencies);

			auto nodeReal = std::dynamic_pointer_cast<SimNode<Real>>(topNode);
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
		void addComponent(IdentifiedObject::Ptr component) {
			auto powerCompComplex = std::dynamic_pointer_cast<SimPowerComp<Complex>>(component);
			if (powerCompComplex) powerCompComplex->initialize(mFrequencies);

			auto powerCompReal = std::dynamic_pointer_cast<SimPowerComp<Real>>(component);
			if (powerCompReal) powerCompReal->initialize(mFrequencies);

			mComponents.push_back(component);
		}

		/// Add multiple components
		void addComponents(IdentifiedObject::List components) {
			for (auto comp : components)
				addComponent(comp);
		}

		/// Adds component and initializes frequencies
		void addTearComponent(IdentifiedObject::Ptr component) {
			auto powerCompComplex = std::dynamic_pointer_cast<SimPowerComp<Complex>>(component);
			if (powerCompComplex) powerCompComplex->initialize(mFrequencies);

			auto powerCompReal = std::dynamic_pointer_cast<SimPowerComp<Real>>(component);
			if (powerCompReal) powerCompReal->initialize(mFrequencies);

			mTearComponents.push_back(component);
		}

		/// Add multiple components
		void addTearComponents(IdentifiedObject::List components) {
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
		int checkTopologySubnets(std::unordered_map<typename CPS::SimNode<VarType>::Ptr, int>& subnet);

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
