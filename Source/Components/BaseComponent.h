#ifndef BASECOMPONENT_H
#define BASECOMPONENT_H

#include <string>
#include <iostream>
#include "../Logger.h"
//#include "../Simulation.h"

#include "../MathLibrary.h"

/// Base class for all elements that might be added to the matrix.
class BaseComponent {
	protected:
		std::string mName;
		int mNode1;
		int mNode2;
		int mNode3;

	public:	
		BaseComponent() { }
		BaseComponent(std::string name) { this->mName = name; }
		BaseComponent(int src, int dest) { this->mNode1 = src - 1; this->mNode2 = dest - 1; }
		BaseComponent(std::string name, int src, int dest) { this->mName = name;  this->mNode1 = src - 1; this->mNode2 = dest - 1; }
		BaseComponent(std::string name, int node1, int node2, int node3) { this->mName = name;  this->mNode1 = node1 - 1; this->mNode2 = node2 - 1; this->mNode3 = node3 - 1; }
		virtual ~BaseComponent() { }

		int getNode1() { return mNode1; }
		int getNode2() { return mNode2; }
		std::string getName() { return mName; }

		virtual void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) = 0;
		virtual void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
		virtual void init(double om, double dt) { }
		virtual void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }
		virtual void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) { }


};
#endif