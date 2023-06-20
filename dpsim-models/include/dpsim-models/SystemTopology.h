/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>
#include <algorithm>

#include <dpsim-models/TopologicalPowerComp.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimNode.h>

#ifdef WITH_GRAPHVIZ
  #include <dpsim-models/Graph.h>
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
		SystemTopology(Real frequency)
		: mFrequencies(initFrequency(frequency)),
		  mSystemFrequency(frequency),
		  mSystemOmega(2 * PI * frequency) { }

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
			componentsAtNodeList();
		}

		/// Standard constructor for multi frequency simulations
		SystemTopology(Real frequency, Matrix frequencies, TopologicalNode::List nodes, IdentifiedObject::List components)
		: mFrequencies(frequencies),
		  mSystemFrequency(frequency),
		  mSystemOmega(2 * PI * frequency) {
			addNodes(nodes);
			addComponents(components);
			componentsAtNodeList();
		}

		Matrix initFrequency(Real frequency) const;

		/// Reset state of components
		void reset();

		// #### Add Objects to SystemTopology ####

		/// Adds node and initializes frequencies
		void addNode(TopologicalNode::Ptr topNode);

		/// Adds node at specified position and initializes frequencies
		void addNodeAt(TopologicalNode::Ptr topNode, UInt index);

		/// Add multiple nodes
		void addNodes(const TopologicalNode::List& topNodes);

		/// Adds component and initializes frequencies
		void addComponent(IdentifiedObject::Ptr component);

		/// Connect component to simNodes
		template <typename VarType>
		void connectComponentToNodes(typename SimPowerComp<VarType>::Ptr component, typename SimNode<VarType>::List simNodes);

		void componentsAtNodeList();

		/// Add multiple components
		void addComponents(const IdentifiedObject::List& components);

		/// Initialize nodes and SG power from PowerFlow
		void initWithPowerflow(const SystemTopology& systemPF, CPS::Domain domain = CPS::Domain::DP);

		/// Adds component and initializes frequencies
		void addTearComponent(IdentifiedObject::Ptr component);

		/// Add multiple components
		void addTearComponents(const IdentifiedObject::List& components);

		// #### Get Objects from SystemTopology ####

		/// Returns TopologicalNode by index in node list
		template<typename Type>
		typename std::shared_ptr<Type> node(UInt index);

		/// Returns TopologicalNode by name
		template<typename Type>
		typename std::shared_ptr<Type> node(std::string_view name);

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

		std::map<String, String, std::less<>> listIdObjects() const;

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
		String render();
		void renderToFile(String filename);
#endif

	private:
		template<typename VarType>
		void multiplyPowerComps(Int numberCopies);
	};
}
