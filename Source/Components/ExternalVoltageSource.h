#pragma once

#include "IdealVoltageSource.h"

namespace DPsim {
	/** Ideal voltage source, but the voltage value can be changed between simulation
	 * steps (for example for interfacing with another simulator) */
	class ExternalVoltageSource : public IdealVoltageSource {
	public:
		ExternalVoltageSource() {};

		ExternalVoltageSource(std::string name, int src, int dest, Real initVoltage, Real initPhase, int num);

		void setVoltage(Real real, Real imag);
	};
}
