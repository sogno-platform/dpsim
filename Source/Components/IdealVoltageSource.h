#ifndef IDEALVOLTAGESOURCE_H
#define IDEALVOLTAGESOURCE_H

#include "BaseComponent.h"

namespace DPsim {

	/// Ideal Voltage source model:
	/// This model uses modified nodal analysis to represent an ideal voltage source. 
	/// For a voltage source between nodes j and k, a new variable (current across the voltage source) is added to the left side vector
	/// as unkown and it is taken into account for the equation of node j as positve and for the equation of node k as negative. Moreover
	/// a new equation ej - ek = V is added to the problem.
	class IdealVoltageSource : public BaseComponent {

	protected:

		//  ### Ideal Voltage source parameters ###
		/// Real and imaginary part of the voltage [V]
		Real mVoltageDiffr;
		Real mVoltageDiffi;

		Real mVoltageAtSourcer;
		Real mVoltageAtSourcei;
		Real mVoltageAtDestr;
		Real mVoltageAtDesti;
		Real mCurrentr;
		Real mCurrenti;

		/// Number of voltage source (first, second...)
		int number;

	public:		
		IdealVoltageSource() { ; };

		/// define paramenters of the voltage source
		IdealVoltageSource(std::string name, int src, int dest, Real voltage, Real phase, int num);

		void init(Real om, Real dt) { }
		
		/// Inserts the current across the voltage source in the equations of node j and k and add the equantion ej - ek = V to the problem
		void applySystemMatrixStamp(SystemModel& system);
		
		/// Stamps voltage source to the current vector
		void applyRightSideVectorStamp(SystemModel& system);
		
		/// Stamps voltage source to the current vector
		void step(SystemModel& system, Real time);
		
		void postStep(SystemModel& system) { }

		virtual Complex getCurrent(SystemModel& system);
	};
}
#endif
