/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <graphviz/gvc.h>

#include <dpsim-models/Graph.h>

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

  mGraph = agopen((char *)name.c_str(), desc, NULL);
  mPtr = mGraph;
  mKind = AGRAPH;
}

CPS::Graph::Graph::~Graph() { agclose(mGraph); }

void CPS::Graph::Graph::render(std::ostream &os, const CPS::String &layout,
                               const CPS::String &format) {
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

Node *CPS::Graph::Graph::addNode(const CPS::String &name) {
  auto n = new Node(this, name);

  mNodes[name] = n;

  return n;
}

Edge *CPS::Graph::Graph::addEdge(const CPS::String &name, Node *head,
                                 Node *tail) {
  auto e = new Edge(this, name, head, tail);

  mEdges[name] = e;

  return e;
}

Node *CPS::Graph::Graph::node(const CPS::String &name) { return mNodes[name]; }

Edge *CPS::Graph::Graph::edge(const CPS::String &name) { return mEdges[name]; }

void CPS::Graph::Element::set(const CPS::String &key, const CPS::String &value,
                              bool html) {
  Agraph_t *g = agraphof(mPtr);

  char *d = (char *)"";
  char *k = (char *)key.c_str();
  char *v = (char *)value.c_str();
  char *vd = html ? agstrdup_html(g, v) : agstrdup(g, v);

  agsafeset(mPtr, k, vd, d);
}

CPS::Graph::Node::Node(Graph *g, const String &name) {
  mNode = agnode(g->mGraph, (char *)name.c_str(), 1);
  mPtr = mNode;
  mKind = AGNODE;
}

CPS::Graph::Edge::Edge(Graph *g, const CPS::String &name, Node *head,
                       Node *tail) {
  mEdge = agedge(g->mGraph, head->mNode, tail->mNode, (char *)name.c_str(), 1);
  mPtr = mEdge;
  mKind = AGEDGE;
}
