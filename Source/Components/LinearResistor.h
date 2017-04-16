#ifndef LINEARRESISTOR_H
#define LINEARRESISTOR_H

#include <iostream>
#include "BaseComponent.h"

namespace DPsim {

	class LinearResistor : public BaseComponent {
	protected:

		///Resistance [ohm]
		Real mResistance;
		
		///Conductance [S]
		Real mConductance;

		///Real Part of the voltage at node 1 [V]
		Real mVoltageAtNode1Re;

		///Imaginary Part of the voltage at node 1 [V]
		Real mVoltageAtNode1Im;

		///Real Part of the voltage at node 2 [V]
		Real mVoltageAtNode2Re;

		///Imaginary Part of the voltage at node 2 [V]
		Real mVoltageAtNode2Im;

	public:
		LinearResistor() { ; };
		LinearResistor(std::string name, int src, int dest, Real resistance);

		void init(Real om, Real dt) { }
		void applySystemMatrixStamp(SystemModel& system);
		void applyRightSideVectorStamp(SystemModel& system) { }
		void step(SystemModel& system, Real time) { }
		void postStep(SystemModel& system) { }
	};
}
#endif
