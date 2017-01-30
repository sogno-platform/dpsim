#ifndef SYNCHRONGENERATOREMT_H
#define SYNCHRONGENERATOREMT_H

#include "../Components.h"

/// Synchronous generator model
/// If parInPerUnit is not set, the parameters have to be given with their respective stator or rotor 
/// referred values. The calculation to per unit is performed in the initialization.
/// The case where parInPerUnit is not set will be implemented later.
/// parameter names include underscores and typical variables names found in literature instead of
/// descriptive names in order to shorten formulas and increase the readability

class SynchronGeneratorEMT : public BaseComponent {
	protected:
				
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
		/// unsaturated d-axis mutual inductance Lmd [H]
		double mLmd0;
		/// q-axis mutual inductance Lmq [H]
		double mLmq;	
		/// unsaturated q-axis mutual inductance Lmq [H]
		double mLmq0;
		/// field resistance Rfd [Ohm]
		double mRfd;
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
		/// q winding inductance
		double mLaq;
		/// d winding inductance
		double mLad;
						
		/// inertia J [kg*m^2]
		double mJ;
		/// number of poles
		int mPoleNumber;
		/// inertia coefficient H
		double mH;

		// ### Stator base values ###
		/// specifies if the machine parameters are transformed to per unit
		SynchGenStateType mStateType;
		/// base stator voltage
		double mBase_v;
		/// base stator voltage RMS
		double mBase_V_RMS;
		/// base stator current
		double mBase_i;
		/// base stator current RMS
		double mBase_I_RMS;
		/// base stator impedance
		double mBase_Z;
		/// base electrical angular frequency
		double mBase_OmElec;
		/// base mechanical angular frequency
		double mBase_OmMech;
		/// base stator inductance
		double mBase_L;
		/// base torque
		double mBase_T;
		/// base flux linkage
		double mBase_Psi;

		/// base field current
		double mBase_ifd;
		/// base field voltage
		double mBase_vfd;
		/// base field impedance
		double mBase_Zfd;
		/// base field inductance
		double mBase_Lfd;
		

		// ### State variables ###
		/// rotor speed omega_r
		double mOmMech;
		/// theta
		double mThetaMech;
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
		/// interface voltage vector abcs
		DPSMatrix mAbcsVoltages = DPSMatrix::Zero(3, 1);
		/// interface current vector abcs
		DPSMatrix mAbcsCurrents = DPSMatrix::Zero(3, 1);
		/// interface voltage vector dq0
		DPSMatrix mDq0Voltages = DPSMatrix::Zero(3, 1);
		/// interface current vector dq0
		DPSMatrix mDq0Currents = DPSMatrix::Zero(3, 1);
		
		// ### Useful Matrices ###
		/// inductance matrix
		DPSMatrix mInductanceMat = DPSMatrix::Zero(7, 7);
		/// resistance matrix
		DPSMatrix mResistanceMat = DPSMatrix::Zero(7, 7);
		/// reactance matrix
		DPSMatrix mReactanceMat = DPSMatrix::Zero(7, 7);
		/// omega - flux matrix
		DPSMatrix mOmegaFluxMat = DPSMatrix::Zero(7, 7);	
		/// matrix for reversing stator current directions in calculations with respect to other currents
		DPSMatrix mReverseCurrents = DPSMatrix::Zero(7, 7);
		
	public:
		SynchronGeneratorEMT() { };
		void applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) { }
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) { }
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) { }

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit or 
		/// stator referred parameters depending on the setting of parameter type.
		/// The initialization mode depends on the setting of state type.
		SynchronGeneratorEMT(std::string name, int node1, int node2, int node3,
			SynchGenStateType stateType, double nomPower, double nomVolt, double nomFreq, int poleNumber, double nomFieldCur,
			SynchGenParamType paramType, double Rs, double Ll, double Lmd, double Lmd0, double Lmq, double Lmq0,
			double Rfd, double Llfd, double Rkd, double Llkd,
			double Rkq1, double Llkq1, double Rkq2, double Llkq2,
			double inertia);		

		/// Initializes the per unit or stator referred machine parameters with the machine parameters given in per unit.
		/// The initialization mode depends on the setting of state type.
		void InitWithPerUnitParam(
			double Rs, double Ll, double Lmd, double Lmd0, double Lmq, double Lmq0,
			double Rfd, double Llfd, double Rkd, double Llkd, double Rkq1, double Llkq1,
			double Rkq2, double Llkq2,
			double H);

		/// Not finished yet.
		void InitWithStatorRefParam(
			double Rs, double Ll, double Lmd, double Lmd0, double Lmq, double Lmq0,
			double Rfd, double Llfd, double Rkd, double Llkd,
			double Rkq1, double Llkq1, double Rkq2, double Llkq2,
			double J);

		/// Initializes states in per unit or stator referred variables depending on the setting of the state type. 
		/// Function parameters have to be given in real units.
		void Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt,
			double initActivePower, double initReactivePower, double initTerminalVolt, double initVoltAngle);
		
		/// Initializes states in per unit. All machine parameters are assumed to be in per unit.
		/// Function parameters have to be given in real units.
		void InitStatesInPerUnit(double initActivePower, double initReactivePower,
			double initTerminalVolt, double initVoltAngle);

		/// Not finished yet
		void InitStatesInStatorRefFrame(double initActivePower, double initReactivePower,
			double initTerminalVolt, double initVoltAngle);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector.
		void Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t, double fieldVoltage, double mechPower);

		/// Performs an Euler forward step with the state space model of a synchronous generator 
		/// to calculate the flux and current from the voltage vector in per unit.
		void StepInPerUnit(double om, double dt, double t, double fieldVoltage, double mechPower);

		/// Not finished yet.
		void StepInStatorRefFrame(double om, double dt, double t, double fieldVoltage, double mechPower);
					
		/// Retrieves calculated voltage from simulation for next step
		void PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t);

		/// Park transform as described in Krause
		DPSMatrix ParkTransform(double theta, DPSMatrix& in);

		/// Inverse Park transform as described in Krause
		DPSMatrix InverseParkTransform(double theta, DPSMatrix& in);

		DPSMatrix GetVoltages() { return mVoltages; }
		DPSMatrix GetCurrents() { return mCurrents; }
		DPSMatrix GetFluxes() { return mFluxes; }
};
#endif
