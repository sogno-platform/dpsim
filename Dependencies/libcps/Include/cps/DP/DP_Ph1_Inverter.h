/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <map>

#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief Single phase inverter model
	///
	/// add more explanation here regarding bessel function model
	class Inverter :
		public PowerComponent<Complex>,
		public MNAInterface,
		public SharedFactory<Inverter> {
	protected:
		// #### Model specific variables ####
		/// DC bus voltage
		Real mVin = 360;
		/// system frequency (should be updated every step)
		Real mFreqMod = 50;
		/// system angular frequency
		Real mOmMod = 2.*PI * mFreqMod;
		/// switching frequency (constant)
		Real mFreqCar = 10e3;
		/// switching angular frequency
		Real mOmCar = 2.*PI * mFreqCar;
		/// Modulation Index
		Real mModIdx = 0.87;
		//mMr = sqrt(2)*mV_grid/mV_in;
		/// Carrier phase
		Real mPhaseCar = 0;
		/// Modulation phase
		Real mPhaseMod = 0;

		/// Number of harmonics
		UInt mHarNum;
		/// Maximum number of carrier signal harmonics
		Int mMaxCarrierHarm = 2;
		/// Maximum number of modulation signal harmonics
		Int mMaxModulHarm = 3;
		///
		UInt mCarHarNum;
		///
		UInt mModHarNum;
		///
		MatrixInt mHarmMap;
		/// Maximum upper limit for Bessel function 1st kind summation
		Int mMaxBesselSumIdx = 20;
		/// Vector of carrier signal harmonics
		std::vector<Int> mCarHarms;
		/// Vector of modulation signal harmonics
		std::vector<Int> mModHarms;

		/// voltage part of system fundamental
		Real mVfund = 0;
		/// Vector of phasor frequencies
		Matrix mPhasorFreqs;
		/// Vector of phasor magnitudes
		Matrix mPhasorMags;
		/// Vector of phasor phases
		Matrix mPhasorPhases;
		///
		std::vector<Real> mFactorials;
		///
		std::map<Int,Real> mMultInvFactorials;

		void generateFrequencies();

		// #### Math functions ####

		/// Bessel function
		Real besselFirstKind_n(Int n, Int k_max, Real x) {
			Real Jn = 0;
			for (Int k = 0; k <= k_max; k++) {
				Real Jn_k = pow(-1,k) / factorial(k) * multInvFactorial(k+n) * pow(x/2., 2.*k+n);
				Jn = Jn + Jn_k;
				//mSLog->info("Jn_n = {:f}", Jn_n);
			}
  			return Jn;
		}

		/// Bessel function using look up tables for factorials
		Real besselFirstKind_n_opt(Int n, Int k_max, Real x) {
			Real Jn = 0;
			for (Int k = 0; k <= k_max; k++) {
				Real Jn_k = pow(-1,k) / mFactorials[k] * mMultInvFactorials[k+n] * pow(x/2., 2.*k+n);
				Jn = Jn + Jn_k;
			}
			return Jn;
		}

		long long factorial(Int n) {
			return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
		}

		Real multInvFactorial(Int n) {
			if(n < 0) return 0;
			else return 1. / factorial(n);
		}

		Real multInvIntGamma(Real n) {
			if(n <= 0) return 0;
			else return 1./std::tgamma(n);
		}

	public:
		/// Defines UID, name and logging level
		Inverter(String name, String uid, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		Inverter(String name, Logger::Level logLevel = Logger::Level::off)
			: Inverter(name, name, logLevel) { }

		// #### General ####
		///
		void initializeFromPowerflow(Real frequency);
		///
		void initialize(Matrix frequencies);
		///
		void setParameters(std::vector<Int> carrierHarms, std::vector<Int> modulHarms,
			Real inputVoltage, Real ratio, Real phase);
		///
		void calculatePhasors();

		// #### MNA Functions ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		void mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		void mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& rightVector);
		void mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx);

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(Inverter& inverter) :
				Task(inverter.mName + ".MnaPreStep"), mInverter(inverter) {
				mModifiedAttributes.push_back(mInverter.attribute("right_vector"));
				mModifiedAttributes.push_back(mInverter.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Inverter& mInverter;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(Inverter& inverter, Attribute<Matrix>::Ptr leftVector) :
				Task(inverter.mName + ".MnaPostStep"),
				mInverter(inverter), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mInverter.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Inverter& mInverter;
			Attribute<Matrix>::Ptr mLeftVector;
		};

		class MnaPreStepHarm : public CPS::Task {
		public:
			MnaPreStepHarm(Inverter& inverter) :
				Task(inverter.mName + ".MnaPreStepHarm"),
				mInverter(inverter) {
				mModifiedAttributes.push_back(mInverter.attribute("right_vector"));
				mModifiedAttributes.push_back(mInverter.attribute("v_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Inverter& mInverter;
		};

		class MnaPostStepHarm : public CPS::Task {
		public:
			MnaPostStepHarm(Inverter& inverter, std::vector<Attribute<Matrix>::Ptr> leftVectors) :
				Task(inverter.mName + ".MnaPostStepHarm"),
				mInverter(inverter), mLeftVectors(leftVectors) {
				for (UInt i = 0; i < mLeftVectors.size(); i++)
					mAttributeDependencies.push_back(mLeftVectors[i]);
				mModifiedAttributes.push_back(mInverter.attribute("i_intf"));
			}
			void execute(Real time, Int timeStepCount);
		private:
			Inverter& mInverter;
			std::vector< Attribute<Matrix>::Ptr > mLeftVectors;
		};

	};
}
}
}
