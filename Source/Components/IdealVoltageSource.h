#ifndef IDEALVOLTAGESOURCE_H
#define IDEALVOLTAGESOURCE_H

#include "BaseComponent.h"

namespace DPsim {

	class IdealVoltageSource : public BaseComponent {

		/// Ideal Voltage source model:
		/// This model uses modified nodal analysis to represent an ideal voltage source. 
		/// For a voltage source between nodes j and k, a new variable (current across the voltage source) is added to the left side vector
		///as unkown and it is taken into account for the equation of node j as positve and for the equation of node k as negative. Moreover
		///a new equation ej - ek = V is added to the problem.

	protected:

		//  ### Ideal Voltage source parameters ###
		/// Real and imaginary part of the voltage [V]
		Real mVoltageDiffr;
		Real mVoltageDiffi;

		/// Number of voltage source (first, second...)
		int number;
		

	public:
		
		IdealVoltageSource() { ; };

		/// define paramenters of the voltage source
		IdealVoltageSource(std::string name, int src, int dest, Real voltage, Real phase, int num);

		/// Inserts the current across the voltage source in the equations of node j and k and add the equantion ej - ek = V to the problem
		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);

		/// Stamps voltage source to the current vector
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);

		/// does nothing
		void init(int compOffset, Real om, Real dt) { }

		/// Stamps voltage source to the current vector
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);

		/// does nothing
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};
}
#endif
