#ifndef INTERFACECURRENTSOURCE_H
#define INTERFACECURRENTSOURCE_H

#include <string>
#include <iostream>

#include "../MathLibrary.h"

/// Base class for all elements that might be added to the matrix.
class InterfaceCurrentSource {
	protected:
		std::string name;
		int node1;
		int node2;

	public:
		InterfaceCurrentSource() { }
		InterfaceCurrentSource(int src, int dest) { this->node1 = src - 1; this->node2 = dest - 1; }
		InterfaceCurrentSource(std::string name, int src, int dest) { this->name = name;  this->node1 = src - 1; this->node2 = dest - 1; }
		virtual ~InterfaceCurrentSource() { }

		int getNode1() { return node1; }
		int getNode2() { return node2; }
		virtual void Init(int compOffset, double om, double dt) { }
		virtual void Step(int compOffset, double om, double dt, double t) { }
		virtual void PostStep(int compOffset, double om, double dt, double t) { }
};
#endif
