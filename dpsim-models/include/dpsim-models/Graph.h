/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <map>

#include <graphviz/cgraph.h>

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Graph {

enum class Type { undirected, directed };

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

/**
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

  void render(std::ostream &os, const String &layout = "dot",
              const String &format = "svg");

  Node *addNode(const String &name);
  Edge *addEdge(const String &name, Node *head, Node *tail);

  Node *node(const String &name);
  Edge *edge(const String &name);
};
} // namespace Graph
} // namespace CPS
