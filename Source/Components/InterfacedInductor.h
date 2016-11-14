#ifndef INTERFACEDINDUCTOR_H
#define INTERFACEDINDUCTOR_H

#include <iostream>
#include "InterfaceCurrentSource.h"

class InterfacedInductor : public InterfaceCurrentSource {
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
		
		void Init(int compOffset, double om, double dt);
		void Step(DPSMatrix& j, int compOffset, double om, double dt, double t);

};
#endif
