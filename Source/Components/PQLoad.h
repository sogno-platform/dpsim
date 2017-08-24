#ifndef PQLOAD_H
#define PQLOAD_H

#include "RxLine.h"

namespace DPsim {
	// TODO currently modeled as an impedance, which obviously doesn't have a constant power characteristic
	class PQLoad : public RxLine {
	protected:
		Real mActivePower;
		Real mReactivePower;
		Real mSvVoltage;
	public:
		PQLoad(std::string name, int src, int dest, Real p, Real q, Real volt, Real angle);
		void init(Real om, Real dt);
	};
};

#endif
