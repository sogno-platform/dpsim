#ifndef INTERFACEDINDUCTOR_H
#define INTERFACEDINDUCTOR_H

#include "BaseComponent.h"

/// Synchronous generator model
/// all parameters are referred to the stator and variables are in rotor reference frame


class SynchronousGenerator : public BaseComponent {
	protected:
		/// nominal power Pn [VA]
		double mNomPower;
		/// nominal voltage Vn [V] (RMS)
		double mNomVolt;
		/// nominal frequency fn [Hz]
		double mNomFreq;
		/// nominal field current Ifn [A]
		double mNomFieldCur;
		/// stator resistance Rs[Ohm]
		double mStatorRes;
		/// leakage inductance Ll [H]
		double mLeakInd;
		/// d-axis mutual inductance Lmd [H]
		double mMutInd_d;
		/// q-axis mutual inductance Lmq [H]
		double mMutInd_q;
		/// field resistance Rfd [Ohm]
		double mFieldRes;
		/// field leakage inductance Llfd [H]
		double mFieldLeakInd;
		/// d-axis damper resistance Rkd [Ohm]
		double mDampRes_d;
		/// d-axis damper leakage inductance Llkd [H]
		double mDampLeakInd_d;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		double mDampRes1_q;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		double mDampLeakInd1_q;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		double mDampRes2_q;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		double mDampLeakInd2_q;
		/// inertia J [kg*m^2]
		double mInertia;
		/// number of poles
		int mPoleNumber;

	public:
		SynchronousGenerator() { };
		SynchronousGenerator(std::string name, int src, int dest, double statorRes, double leakInd, double mutInd_d, 
			double mutInd_q, double fieldRes, double fieldLeakInd, double dampRes_d, double dampLeakInd_d, double dampRes1_q, double dampLeakInd1_q,
			double dampRes2_q, double dampLeakInd2_q, double inertia, int poleNumber);
		
		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t);
		void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);

};
#endif
