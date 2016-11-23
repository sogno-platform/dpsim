#ifndef INTERFACEDINDUCTOR_H
#define INTERFACEDINDUCTOR_H

#include "BaseComponent.h"

class InterfacedInductor : public BaseComponent {
	protected:
		double inductance;
		double voltageRe;
		double voltageIm;
		double currentRe;
		double currentIm;
		double currentStepRe;
		double currentStepIm;

	public:
		InterfacedInductor() { };
		InterfacedInductor(std::string name, int src, int dest, double inductance);
		
		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		virtual void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);

};
#endif
