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

		/// Component name
		std::string mName;

		/// Component node 1
		int mNode1;

		/// Component node 2
		int mNode2;

		/// Component node 3
		int mNode3;

	public:
		BaseComponent() { }
		BaseComponent(std::string name) { this->mName = name; }
		BaseComponent(int src, int dest) { this->mNode1 = src - 1; this->mNode2 = dest - 1; }
		BaseComponent(std::string name, int src, int dest) { this->mName = name;  this->mNode1 = src - 1; this->mNode2 = dest - 1; }
		BaseComponent(std::string name, int node1, int node2, int node3) { this->mName = name;  this->mNode1 = node1 - 1; this->mNode2 = node2 - 1; this->mNode3 = node3 - 1; }
		virtual ~BaseComponent() { }

		/// get value of node1
		int getNode1() { return mNode1; }
		/// get value of node2
		int getNode2() { return mNode2; }
		/// get value of node3
		int getNode3() { return mNode3; }

		std::string getName() { return mName; }

		/// Initializes variables of components
		virtual void init(Real om, Real dt) { }

		/// Stamps conductance matrix
		virtual void applySystemMatrixStamp(SystemModel& system) = 0;

		/// Stamps current source vector
		virtual void applyRightSideVectorStamp(SystemModel& system) { }

		/// Upgrade values on the current source vector
		virtual void step(SystemModel& system, Real time) { }

		/// Upgrade variable values based on the solution of the step
		virtual void postStep(SystemModel& system) { }

		/// Return the current flowing through this component in the previous timestep
		virtual Complex getCurrent(SystemModel& system) { }
	};
}

#endif
