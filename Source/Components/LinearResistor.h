#ifndef LINEARRESISTOR_H
#define LINEARRESISTOR_H

#include <iostream>

#include "CircuitElement.h"

class LinearResistor : public CircuitElement
{
	protected:
		double resistance;
		double conductance;
		double voltageAtSourcer;
		double voltageAtSourcei;

		double voltageAtDestr;
		double voltageAtDesti;

	public:
		LinearResistor() {;};
		LinearResistor(std::string name, int src, int dest, double resistance);
		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
};
#endif
