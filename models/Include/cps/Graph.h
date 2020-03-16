/** Wrapper around Graphviz's cgraph library
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <map>

#include <graphviz/cgraph.h>

#include <cps/Definitions.h>

namespace CPS {
namespace Graph {

	enum class Type {
		undirected,
		directed
	};

	class Graph;

	class Element {

	protected:
		void *mPtr;
		int mKind;

	public:
		void set(const String &key, const String &value, bool html = false);
	};

	class Node : public Element {
	public:
		Agnode_t *mNode;

		Node(Graph *g, const String &name);
	};

	class Edge : public Element {
	public:
		Agedge_t *mEdge;

		Edge(Graph *g, const String &name, Node *head, Node *tail);
	};

	/** Wrapper around Graphviy Cgraph library
	 *
	 * See: http://www.graphviz.org/pdf/libguide.pdf
	 */
	class Graph : public Element {

	protected:
		std::map<String, Node *> mNodes;
		std::map<String, Edge *> mEdges;

	public:
		Agraph_t *mGraph;

		Graph(const String &name, Type type, bool strict = false);
		~Graph();

		void render(std::ostream &os, const String &layout = "dot", const String &format = "svg");

		Node * addNode(const String &name);
		Edge * addEdge(const String &name, Node *head, Node *tail);

		Node * node(const String &name);
		Edge * edge(const String &name);
	};
}
}
