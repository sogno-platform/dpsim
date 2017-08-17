#ifndef PQLOAD_H
#define PQLOAD_H

#include "BaseComponent.h"

namespace DPsim {
	class PQLoad : public BaseComponent {
	protected:
		Real mActivePower;
		Real mReactivePower;
	public:
		PQLoad(std::string name, int src, int dest, Real p, Real q);

		// TODO actually implement these methods
		void init(Real om, Real dt) {}
		void applySystemMatrixStamp(SystemModel& system) {}
		void applyRightSideVectorStamp(SystemModel& system) {}
		void step(SystemModel& system, Real time) {}
		void postStep(SystemModel& system) {}
	};
};

#endif
