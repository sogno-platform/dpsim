#ifndef VOLTSOURCERES_H
#define VOLTSOURCERES_H

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceRes : public BaseComponent {

		/// Real Voltage source:
		/// The real voltage source is a voltage source in series with a resistance, which is transformed to a current source with
		/// a parallel resistance using the Norton equivalent

	protected:

		//  ### Real Voltage source parameters ###
		/// Real and imaginary part of the voltage [V]
		Real mVoltageDiffr;
		Real mVoltageDiffi;
		Real mVoltageAtSourcer;
		Real mVoltageAtSourcei;
		Real mVoltageAtDestr;
		Real mVoltageAtDesti;

		/// Resistance [ohm]
		Real mResistance;

		/// conductance [S]
		Real mConductance;

		/// equivalent current source [A]
		Real mCurrentr;
		Real mCurrenti;

	public:
		VoltSourceRes() { ; };

		/// define voltage source paramenters
		VoltSourceRes(std::string name, int src, int dest, Real voltage, Real phase, Real resistance);

		/// Stamps voltage source resistance to the conductance matrix
		void applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt);

		/// Stamps equivalent current source to the current vector 
		void applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt);

		/// does nothing
		void init(int compOffset, Real om, Real dt) { }

		/// Stamps equivalent current source to the current vector 
		void step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t);

		/// does nothing
		void postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, Real om, Real dt, Real t) { }
	};
}
#endif
