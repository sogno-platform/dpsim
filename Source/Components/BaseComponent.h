#ifndef BASECOMPONENT_H
#define BASECOMPONENT_H

#include <string>
#include <iostream>

#include "../Logger.h"
#include "../SystemModel.h"
#include "../MathLibrary.h"

namespace DPsim {

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

		virtual void init(Real om, Real dt) { }
		virtual void applySystemMatrixStamp(SystemModel& system) = 0;
		virtual void applyRightSideVectorStamp(SystemModel& system) { }		
		virtual void step(SystemModel& system, Real time) { }
		virtual void postStep(SystemModel& system) { }
	};
}
#endif