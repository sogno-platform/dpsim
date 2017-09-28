#ifndef BASECOMPONENT_H
#define BASECOMPONENT_H

#include <string>
#include <iostream>

#include "../SystemModel.h"
#include "../MathLibrary.h"

namespace DPsim {

	enum AttrType {
		AttrReal,
		AttrInt,
		AttrString, // value should be *std::string, not *char!
		AttrComplex
	};

	struct CompAttr {
		AttrType type;
		void* value;
	};

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
		/// Component logger control for internal variables
		bool mLogActive;

		/// Map of all attributes that should be exported to the Python interface
		std::map<std::string, CompAttr> attrMap;

	public:
		BaseComponent() { }

		BaseComponent(std::string name, int src, int dest, bool logActive = false) {
			this->mName = name;
			this->mNode1 = src - 1;
			this->mNode2 = dest - 1;
			this->mLogActive = logActive;
			attrMap["name"] = {AttrString, &this->mName};
			attrMap["node1"] = {AttrInt, &this->mNode1};
			attrMap["node2"] = {AttrInt, &this->mNode2};
		}

		BaseComponent(std::string name, int node1, int node2, int node3, bool logActive = false) :
			BaseComponent(name, node1, node2, logActive) {
			this->mNode3 = node3 - 1;
			attrMap["node3"] = {AttrInt, &this->mNode3};
		}
		virtual ~BaseComponent() { }

		/// get value of node1
		int getNode1() { return mNode1; }
		/// get value of node2
		int getNode2() { return mNode2; }
		/// get value of node3
		int getNode3() { return mNode3; }

		std::map<std::string, CompAttr>& getAttrMap() { return attrMap; }

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
		virtual Complex getCurrent(SystemModel& system) {
			std::cerr << "getCurrent implementation missing" << std::endl;
			std::exit(1);
			return Complex(0, 0);
		}

	};
}

#endif
