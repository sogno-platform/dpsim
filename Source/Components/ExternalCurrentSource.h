#pragma once

#include "CurrentSource.h"

namespace DPsim {
	/** Ideal current source, but the current value can be changed between simulation
	 * steps (for example for interfacing with another simulator) */
	class ExternalCurrentSource : public CurrentSource {
	private:
		Real mPhase;
	
	public:
		ExternalCurrentSource() {};

		ExternalCurrentSource(std::string name, int src, int dest, Real current, Real phase);

		void setCurrent(Real current);
	};
}
