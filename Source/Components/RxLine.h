#ifndef RXLINE_H
#define RXLINE_H

#include "BaseComponent.h"

namespace DPsim {

	class RxLine : public BaseComponent {
	protected:

		double resistance;
		double conductance;
		double voltageAtSourcer;
		double voltageAtSourcei;
		double voltageAtDestr;
		double voltageAtDesti;

		double inductance;
		double deltavr;
		double deltavi;
		double currr;
		double curri;
		double cureqr;
		double cureqi;
		double glr, gli;
		double pr, pi;

		int newnode;


	public:
		RxLine() { };
		RxLine(std::string name, int src, int dest, int newNode, double resistance, double inductance);

		void init(double om, double dt);
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system);
		void postStep(SystemModel& system);
	};
}
#endif