#ifndef CAPACITOR_H
#define CAPACITOR_H

#include <iostream>

#include "CircuitElement.h"

class Capacitor : public CircuitElement {
	protected:
		double capacitance;
		double deltavr;
		double deltavi;
		double currr;
		double curri;
		double cureqr;
		double cureqi;	

	public:
		Capacitor() { };
		Capacitor(std::string name, int src, int dest, double capacitance);

		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om,  double dt, double t);
	};
#endif