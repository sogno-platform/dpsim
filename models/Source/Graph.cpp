/** Wrapper around Graphviz's cgraph library
 *
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

#include <graphviz/gvc.h>

#include <cps/Graph.h>

using namespace CPS;
using namespace CPS::Graph;

CPS::Graph::Graph::Graph(const CPS::String &name, Type type, bool strict) {
	Agdesc_t desc;

	switch (type) {
		case Type::directed:
			desc = strict ? Agstrictdirected : Agdirected;
			break;
		case Type::undirected:
			desc = strict ? Agstrictundirected : Agundirected;
			break;
	}

	mGraph = agopen((char *) name.c_str(), desc, NULL);
	mPtr = mGraph;
	mKind = AGRAPH;
}

CPS::Graph::Graph::~Graph() {
	agclose(mGraph);
}

void CPS::Graph::Graph::render(std::ostream &os, const CPS::String &layout, const CPS::String &format) {
	GVC_t *gvc;
	char *data;
	unsigned len;

	gvc = gvContext();

	gvLayout(gvc, mGraph, layout.c_str());
	gvRenderData(gvc, mGraph, format.c_str(), &data, &len);

	os.write(data, len);

	gvFreeRenderData(data);
	gvFreeLayout(gvc, mGraph);

	gvFreeContext(gvc);
}

Node * CPS::Graph::Graph::addNode(const CPS::String &name) {
	auto n = new Node(this, name);

	mNodes[name] = n;

	return n;
}

Edge * CPS::Graph::Graph::addEdge(const CPS::String &name, Node *head, Node *tail) {
	auto e = new Edge(this, name, head, tail);

	mEdges[name] = e;

	return e;
}

Node * CPS::Graph::Graph::node(const CPS::String &name) {
	return mNodes[name];
}

Edge * CPS::Graph::Graph::edge(const CPS::String &name) {
	return mEdges[name];
}

void CPS::Graph::Element::set(const CPS::String &key, const CPS::String &value, bool html) {
	Agraph_t *g = agraphof(mPtr);

	char *d = (char *) "";
	char *k = (char *) key.c_str();
	char *v = (char *) value.c_str();
	char *vd = html ? agstrdup_html(g, v) : agstrdup(g, v);

	agsafeset(mPtr, k, vd, d);
}

CPS::Graph::Node::Node(Graph *g, const String &name) {
	mNode = agnode(g->mGraph, (char *) name.c_str(), 1);
	mPtr = mNode;
	mKind = AGNODE;
}

CPS::Graph::Edge::Edge(Graph *g, const CPS::String &name, Node *head, Node *tail) {
	mEdge = agedge(g->mGraph, head->mNode, tail->mNode, (char *) name.c_str(), 1);
	mPtr = mEdge;
	mKind = AGEDGE;
}
