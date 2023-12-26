/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Base/Base_VSIControlDQ.h>
#include <dpsim-models/SimNode.h>

namespace CPS {
namespace Base {

	/// @brief Base model of average grid forming inverter
	template <typename VarType>
	class VSIVoltageSourceInverterDQ {
	private:
		/// Component logger
		Logger::Log mLogger;

	protected:
		// ### General Parameters ###
		/// mModelAsCurrentSource=true --> Inverter is modeled as current source, otherwise as voltage source
		Bool mModelAsCurrentSource;
		/// Simulation step
		Real mTimeStep;
		/// Nominal Omega
		Real mOmegaN;

		// ### Inverter Parameters ###
		/// Nominal frequency
		Real mOmegaNom;
		/// Nominal voltage
		Real mVnom;
		/// Voltage d reference
		Real mVdRef;
		/// Voltage q reference
		Real mVqRef;
		/// Active power reference
		Real mPref;

		// ### Inverter Flags ###
		/// Flag for usage of interface resistor Rc
		Bool mWithInterfaceResistor = false;
		/// Flag for control droop usage
		Bool mWithDroop = false;

		// Filter parameters
		Real mLf;
		Real mCf;
		Real mRf;
		Real mRc;

		// ### Inverter Variables ###
		/// Omega
		const Attribute<Real>::Ptr mOmega;
		/// System angle (rotating at nominal omega)
		const Attribute<Real>::Ptr mThetaSys;
		/// Inverter angle (rotating at inverter omega)
		const Attribute<Real>::Ptr mThetaInv;
		/// Voltage/Current as control output after transformation interface
		const Attribute<MatrixComp>::Ptr mSourceValue;
		/// Voltage/Current as control output after transformation interface
		const Attribute<Complex>::Ptr mSourceValue_dq;
		/// Measured voltage in dq reference frame
		const Attribute<Complex>::Ptr mVcap_dq;
		/// Measured current in dq reference frame
		const Attribute<Complex>::Ptr mIfilter_dq;
		/// inverter terminal active power
		const Attribute<Complex>::Ptr mPower;

		// ### Voltage Controller Variables ###

		// #### Controllers ####
		/// Determines if VSI control is activated
		Bool mWithControl = true;
			
		/// Signal component modelling voltage regulator and exciter
		std::shared_ptr<Base::VSIControlDQ> mVSIController;		

    public:
		explicit VSIVoltageSourceInverterDQ(Logger::Log Log, CPS::AttributeList::Ptr attributeList,
			Bool modelAsCurrentSource, Bool withInterfaceResistor) :
			mLogger(Log),
			mModelAsCurrentSource(modelAsCurrentSource),
			mWithInterfaceResistor(withInterfaceResistor),
			mOmega(attributeList->create<Real>("Omega", 0)),
			mThetaSys(attributeList->create<Real>("ThetaSys", 0)),
			mThetaInv(attributeList->create<Real>("ThetaInv", 0)),
			mSourceValue(attributeList->create<MatrixComp>("SourceValue", MatrixComp::Zero(1,1))),
			mSourceValue_dq(attributeList->create<Complex>("SourceValue_dq", Complex(0,0))),
			mVcap_dq(attributeList->create<Complex>("Vcap_dq", 0)),
			mIfilter_dq(attributeList->create<Complex>("Ifilter_dq", 0)),
			mPower(attributeList->create<Complex>("Power", 0)){ };

		/// Setter for general parameters of inverter
		void setParameters(Real sysOmega, Real VdRef, Real VqRef);
		/// Setter for filter parameters
		void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);

		// ### Controllers ###
		/// Add VSI Controller
		void addVSIController(std::shared_ptr<Base::VSIControlDQ> VSIController);

	protected:
		///
		virtual void createSubComponents() = 0;
		///
		int determineNumberOfVirtualNodes();
		///
		void initializeFilterVariables(const Complex & interfaceVoltage, 
									   const Complex & interfaceCurrent,
									   typename SimNode<VarType>::List virtualNodesList);
    };
}
}
