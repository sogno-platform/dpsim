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
		double currentr;
		double currenti;

	public:
		VoltageSourceWithResistance() {;};
		VoltageSourceWithResistance(std::string name, int src, int dest, double voltage, double phase, double resistance);

		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt);
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt);
		void init(int compOffset, double om, double dt) { }
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) { }
};
#endif
