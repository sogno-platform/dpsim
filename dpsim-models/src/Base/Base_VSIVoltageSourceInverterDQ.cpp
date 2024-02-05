/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_VSIVoltageSourceInverterDQ.h>

using namespace CPS;

template <typename VarType>
void Base::VSIVoltageSourceInverterDQ<VarType>::setParameters(
	Real sysOmega, Real VdRef, Real VqRef) {

	mOmegaNom = sysOmega;
	mVdRef = VdRef;
	mVqRef = VqRef;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nGeneral Parameters:"
		"\n\tNominal Omega = {} [1/s]"
		"\n\tVdRef = {} [V] "
		"\n\tVqRef = {} [V]",
		mOmegaNom, mVdRef, mVqRef);
}

template <typename VarType>
void Base::VSIVoltageSourceInverterDQ<VarType>::setFilterParameters(
	Real Lf, Real Cf, Real Rf, Real Rc) {

	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;

	createSubComponents();

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nFilter Parameters:"
		"\n\tInductance Lf = {} [H]"
		"\n\tCapacitance Cf = {} [F]"
		"\n\tResistance Rf = {} [H]" 
		"\n\tResistance Rc = {} [F]",
		mLf, mCf, mRf, mRc);
	mLogger->flush();
}

template <typename VarType>
int Base::VSIVoltageSourceInverterDQ<VarType>::determineNumberOfVirtualNodes() {
	// first virtual node is the second node of the rl element
	int numberOfVirtualNodes = 1;
	
	if (mWithInterfaceResistor) 
		numberOfVirtualNodes += 1;	
	
	return numberOfVirtualNodes;
}

template <typename VarType>
void Base::VSIVoltageSourceInverterDQ<VarType>::addVSIController(std::shared_ptr<Base::VSIControlDQ> VSIController) {
	mVSIController = VSIController;
	mWithControl = true;
}

template <typename VarType>
void Base::VSIVoltageSourceInverterDQ<VarType>::initializeFilterVariables(
	const Complex & interfaceVoltage, const Complex & interfaceCurrent,
	typename SimNode<VarType>::List virtualNodesList) {

	// derive initialization quantities of filter
	/// initial filter capacitor voltage
	Complex vcInit;
	if (mWithInterfaceResistor)
		vcInit = interfaceVoltage + interfaceCurrent * mRc;
	else
		vcInit = interfaceVoltage;

	/// initial filter capacitor current 
	Complex icfInit = vcInit * Complex(0., mOmegaNom * mCf);

	/// initial voltage/current equivalent source
	Complex filterCurrentInit = interfaceCurrent + icfInit;
	Complex sourceInitialValue;
	if (mModelAsCurrentSource)
		sourceInitialValue = filterCurrentInit;
	else
		sourceInitialValue = vcInit + filterCurrentInit * Complex(mRf, mOmegaNom * mLf);

	// initialize angles
	**mThetaSys = 0;
	**mThetaInv = std::arg(vcInit);

	// Initialie filter variables in dq domain
	**mVcap_dq = Math::rotatingFrame2to1(vcInit, **mThetaInv, **mThetaSys);
	**mIfilter_dq = Math::rotatingFrame2to1(filterCurrentInit, **mThetaInv, **mThetaSys);
	**mSourceValue_dq = Math::rotatingFrame2to1(sourceInitialValue, **mThetaInv, **mThetaSys);
	
	String inverter_type = mModelAsCurrentSource? "current source": "voltage source";
	String unit = mModelAsCurrentSource? "[A]": "[V]";
	SPDLOG_LOGGER_INFO(mLogger, 
		"\nInverter will be modelled as {}"
		"\nInitialize Filter Variables:"
		"\n\tInitial capacitor voltage: {}[V]"
		"\n\tInitial capacitor voltage d-axis: {}[V]"
		"\n\tInitial capacitor voltage q-axis: {}[V]"
		"\n\tInitial filter current: {}[A]"
		"\n\tInitial filter d-axis: {}[A]"
		"\n\tInitial filter q-axis: {}[A]"
		"\n\tInitial equivalent source: {}{}"
		"\n\tInverter equivalent source d-axis value: {}{}"
		"\n\tInverter equivalent source q-axis value: {}{}",
		inverter_type,
		Logger::phasorToString(vcInit),
		(**mVcap_dq).real(), (**mVcap_dq).imag(),
		Logger::phasorToString(filterCurrentInit),
		(**mIfilter_dq).real(), (**mIfilter_dq).imag(),
		sourceInitialValue, unit,
		(**mSourceValue_dq).real(), unit, 
		(**mSourceValue_dq).imag(), unit);
	mLogger->flush();

	// TODO: MOVE
	// initialize voltage of virtual nodes
	virtualNodesList[0]->setInitialVoltage(vcInit + filterCurrentInit * Complex(mRf, mOmegaNom * mLf));
	if (mWithInterfaceResistor) {
		// filter capacitor is connected to mVirtualNodes[1], the second
		// node of the interface resistor is mTerminals[0]
		virtualNodesList[1]->setInitialVoltage(vcInit);
	}
}

// Declare specializations to move definitions to .cpp
template class CPS::Base::VSIVoltageSourceInverterDQ<Real>;
template class CPS::Base::VSIVoltageSourceInverterDQ<Complex>;