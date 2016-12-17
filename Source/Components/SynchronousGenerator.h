#ifndef SYNCHRONOUSGENERATOR_H
#define SYNCHRONOUSGENERATOR_H

#include "BaseComponent.h"

/// Synchronous generator model
/// all parameters are referred to the stator and variables are in rotor reference frame


class SynchronousGenerator : public BaseComponent {
	protected:
		
		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		bool mUsePerUnit;
		/// base stator voltage
		double mBaseVoltage;
		/// base stator current
		double mBaseCurrent;
		/// base stator impedance
		double mBaseImpedance;
		/// base angular frequency
		double mBaseAngFreq;
		/// base stator inductance
		double mBaseInductance;
		/// base torque
		double mBaseTorque;

		// ### Machine parameters ###
		/// nominal power Pn [VA]
		double mNomPower;
		/// nominal voltage Vn [V] (RMS)
		double mNomVolt;
		/// nominal frequency fn [Hz]
		double mNomFreq;
		/// nominal field current Ifn [A]
		double mNomFieldCur;
		/// stator resistance Rs[Ohm]
		double mRs;
		/// leakage inductance Ll [H]
		double mLl;
		/// d-axis mutual inductance Lmd [H]
		double mLmd;
		/// q-axis mutual inductance Lmq [H]
		double mLmq;
		/// field resistance Rfd [Ohm]
		double mRf;
		/// field leakage inductance Llfd [H]
		double mLlfd;
		/// d-axis damper resistance Rkd [Ohm]
		double mRkd;
		/// d-axis damper leakage inductance Llkd [H]
		double mLlkd;
		/// q-axis damper resistance 1 Rkq1 [Ohm]
		double mRkq1;
		/// q-axis damper leakage inductance 1 Llkq1 [H]
		double mLlkq1;
		/// q-axis damper resistance 2 Rkq2 [Ohm]
		double mRkq2;
		/// q-axis damper leakage inductance 2 Llkq2 [H]
		double mLlkq2;
		/// inertia J [kg*m^2]
		double mInertia;
		/// number of poles
		int mPoleNumber;
		/// inertia coefficient
		double mInertiaCoeff;
		/// q winding inductance
		double mLaq;
		/// d winding inductance
		double mLad;

		// ### State variables ###
		/// rotor speed omega_r
		double mOmega_r;
		/// theta
		double mTheta_r;
		/// mechanical Power Pm [W]
		double mMechPower;
		/// mechanical torque
		double mMechTorque;
		/// electrical torque
		double mElecTorque;
		/// voltage vector q d 0 kq1 kq2 df kd
		DPSMatrix mVoltages = DPSMatrix::Zero(7, 1);
		/// flux linkage vector
		DPSMatrix mFluxes = DPSMatrix::Zero(7, 1);
		/// current vector
		DPSMatrix mCurrents = DPSMatrix::Zero(7 ,1);
		/// inductance matrix
		DPSMatrix mInductanceMat = DPSMatrix::Zero(7, 7);
		/// resistance matrix
		DPSMatrix mResistanceMat = DPSMatrix::Zero(7, 7);
		/// reactance matrix
		DPSMatrix mReactanceMat = DPSMatrix::Zero(7, 7);
		/// omega - flux matrix
		DPSMatrix mOmegaFluxMat = DPSMatrix::Zero(7, 7);
		/// interface voltage vector abcs
		DPSMatrix mAbcsVoltages = DPSMatrix::Zero(3, 1);
		/// interface current vector abcs
		DPSMatrix mAbcsCurrents = DPSMatrix::Zero(3, 1);
		/// interface voltage vector dq0
		DPSMatrix mDq0Voltages = DPSMatrix::Zero(3, 1);
		/// interface current vector dq0
		DPSMatrix mDq0Currents = DPSMatrix::Zero(3, 1);
		
	public:
		SynchronousGenerator() { };
		SynchronousGenerator(std::string name, int node1, int node2, int node3,
			bool usePerUnit, double nomPower, double nomVolt, double nomFreq, double statorRes, double leakInd, double mutInd_d, double mutInd_q,
			double fieldRes, double fieldLeakInd, double dampRes_d, double dampLeakInd_d, double dampRes1_q, double dampLeakInd1_q,
			double dampRes2_q, double dampLeakInd2_q, double inertia, int poleNumber);
		
		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt);
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) { }
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, DPSMatrix initAbcsCurrents, DPSMatrix initAbcsVoltages, double initFieldVoltage, double initFieldCurrent, double initTheta_r);
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t, double fieldVoltage, double fieldCurrent, double mechPower);
		void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);
		DPSMatrix ParkTransform(double theta, DPSMatrix& in);
		DPSMatrix InverseParkTransform(double theta, DPSMatrix& in);

};
#endif
