#ifndef CAPACITOR_H
#define CAPACITOR_H

#include <iostream>

#include "BaseComponent.h"

class Capacitor : public BaseComponent {
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

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) { }
		void init(int compOffset, double om, double dt);
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om,  double dt, double t);
	};
#endif