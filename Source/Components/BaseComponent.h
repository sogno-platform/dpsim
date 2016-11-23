#ifndef CIRCUITELEMENT_H
#define CIRCUITELEMENT_H

#include <string>
#include <iostream>

#include "../MathLibrary.h"

/// Base class for all elements that might be added to the matrix.
class BaseComponent {
	protected:
		std::string name;
		int node1;
		int node2;

	public:	
		BaseComponent() { }
		BaseComponent(int src, int dest) { this->node1 = src - 1; this->node2 = dest - 1; }
		BaseComponent(std::string name, int src, int dest) { this->name = name;  this->node1 = src - 1; this->node2 = dest - 1; }
		virtual ~BaseComponent() { }

		int getNode1() { return node1; }
		int getNode2() { return node2; }
		std::string GetName() { return name; }

		virtual void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) = 0;
		virtual void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) { }
		virtual void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }
		virtual void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om,  double dt, double t) { }
};
#endif