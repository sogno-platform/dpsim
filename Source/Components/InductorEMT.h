#ifndef INDUCTOREMT_H
#define INDUCTOREMT_H

#include "BaseComponent.h"

namespace DPsim {

	class InductorEMT : public BaseComponent {
	protected:
		double mInductance;
		double mDeltav;		
		double mCurr;		
		double mCureq;
		double mGl;
		double mP;

	public:
		InductorEMT() { };
		InductorEMT(std::string name, int src, int dest, double inductance);

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
		void init(double om, double dt);
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);
	};
}
#endif