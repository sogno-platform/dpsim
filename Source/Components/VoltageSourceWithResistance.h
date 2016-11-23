#ifndef VOLTAGESOURCEWITHRESISTANCE_H
#define VOLTAGESOURCEWITHRESISTANCE_H

#include "BaseComponent.h"

class VoltageSourceWithResistance : public BaseComponent {
	protected:
		double voltageDiffr;
		double voltageDiffi;
		double voltageAtSourcer;
		double voltageAtSourcei;
		double voltageAtDestr;
		double voltageAtDesti;
		int internalNodeNum;
		double resistance;
		double conductance;		

	public:
		VoltageSourceWithResistance() {;};
		VoltageSourceWithResistance(std::string name, int src, int dest, double voltage, double phase, double resistance);

		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
};
#endif
