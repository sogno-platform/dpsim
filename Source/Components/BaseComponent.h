#ifndef BASECOMPONENT_H
#define BASECOMPONENT_H

#include <string>
#include <iostream>
#include "../Logger.h"

#include "../MathLibrary.h"

/// Base class for all elements that might be added to the matrix.
class BaseComponent {
	protected:
		std::string name;
		int node1;
		int node2;
		int node3;

	public:	
		BaseComponent() { }
		BaseComponent(std::string name) { this->name = name; }
		BaseComponent(int src, int dest) { this->node1 = src - 1; this->node2 = dest - 1; }
		BaseComponent(std::string name, int src, int dest) { this->name = name;  this->node1 = src - 1; this->node2 = dest - 1; }
		virtual ~BaseComponent() { }

		int getNode1() { return node1; }
		int getNode2() { return node2; }
		std::string getName() { return name; }

		virtual void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) = 0;
		virtual void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
		virtual void init(double om, double dt) { }
		virtual void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }
		virtual void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om,  double dt, double t) { }
};
#endif