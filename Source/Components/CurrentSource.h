#ifndef CURRENTSOURCE_H
#define CURRENTSOURCE_H

#include "BaseComponent.h"

namespace DPsim {

	class CurrentSource : public BaseComponent {
	protected:
		double currentr;
		double currenti;

	public:
		CurrentSource() { ; };
		CurrentSource(std::string name, int src, int dest, double current, double phase);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system);
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
		Complex getCurrent(SystemModel& system);
	};
}
#endif
