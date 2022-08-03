/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SSN_SG4O.h>

using namespace CPS;

EMT::Ph3::SSN::SG4O::SG4O(String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;

	setTerminalNumber(2);
    setVirtualNodeNumber(1);
}

void EMT::Ph3::SSN::SG4O::specificInitialization()
{
	this->timeStep = mTimeStep; //redundant; replace all timeStep in this class with mTimeStep

	State = Matrix::Zero(4, 1);
    yHistory =  Matrix::Zero(4, 1);
	
	**SSN_Function_Result = Matrix::Zero(mRightVector->get().rows(), 1);

    mX_d = mLd;
    mX_dDash = mLd_t;
    mX_q = mLq;
    mX_qDash = mLq_t;
    mT_d0Dash = mTd0_t;
    mT_q0Dash = mTq0_t;
	mE_f = mEf;
	mOmega_base = mBase_OmMech;
	mP_mech = mInitMechPower/((2./3.)*mBase_I*mBase_V);

	nonExplicitState(0 ,0) = **mOmMech;

	Matrix temp = Matrix::Zero(4,1);
	temp(0, 0) = (**mIntfVoltage)(0, 0);
	temp(1, 0) = (**mIntfVoltage)(1, 0);
	temp(2, 0) = (**mIntfVoltage)(2, 0);
	temp(3, 0) = nonExplicitState(0, 0);

	ssnUpdateState();
	ssnUpdateJacobian(temp);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::SG4O::clone(String name) {
	auto copy = SG4O::make(name, mLogLevel);

	return copy;
}

void EMT::Ph3::SSN::SG4O::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), Jacobian(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), -Jacobian(0, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), Jacobian(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), -Jacobian(1, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), Jacobian(2, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), -Jacobian(2, 3));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), Jacobian(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), -Jacobian(0, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), Jacobian(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), -Jacobian(1, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 0), Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 1), Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), Jacobian(2, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(2, 3));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 1), -Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 2), -Jacobian(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 0), -Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 2), -Jacobian(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 0), -Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 1), -Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -Jacobian(2, 2));


		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), -Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), -Jacobian(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), -Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), -Jacobian(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), -Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), -Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -Jacobian(2, 2));
	}
		//Internal Voltages are v1-v0: Positive for Terminal 1, negative for terminal 0. Omega is not a difference: We only have one "virtual terminal" for it.

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(0, 0) ,-Jacobian(3, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(0, 1) ,-Jacobian(3, 1));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(0, 2) ,-Jacobian(3, 2));

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(1, 0) ,Jacobian(3, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(1, 1) ,Jacobian(3, 1));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(1, 2) ,Jacobian(3, 2));

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(3, 3));

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1.);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1.);
}

void EMT::Ph3::SSN::SG4O::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Update internal state
	Real omega = nonExplicitState(0, 0);

	Real U_d = (**mIntfVoltage)(0, 0)*sin(omega)+(**mIntfVoltage)(1, 0)*sin(omega-2.*PI/3.)+(**mIntfVoltage)(2, 0)*sin(omega+2.*PI/3.);
	Real U_q = (**mIntfVoltage)(0, 0)*cos(omega)+(**mIntfVoltage)(1, 0)*cos(omega-2.*PI/3.)+(**mIntfVoltage)(2, 0)*cos(omega+2.*PI/3.);

	yHistory(0, 0) = 0.;
	yHistory(1, 0) = 0.;
	yHistory(2, 0) = 0.;

	yHistory(3, 0)=	State(3, 0) + timeStep*
					((mp_mech+mP_mech)/(4.*mH)
					-U_q*U_q/(4.*mH*mX_qDash)
					+State(1, 0)*U_q/(4.*mH*mX_qDash)
					+U_d*U_d/(4.*mH*mX_dDash)
					+State(2, 0)*U_d/(4.*mH*mX_dDash));
	/*
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), yHistory(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), yHistory(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), yHistory(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), -yHistory(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 1), -yHistory(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 2), -yHistory(2, 0));
	}
	*/

	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), yHistory(3, 0));

	mSLog->debug(
		"\nHistory current term (mnaApplyRightSideVectorStamp): {:s}",
		Logger::matrixToString(yHistory));
	mSLog->flush();
}

void EMT::Ph3::SSN::SG4O::ssnCalculateFunctionResult(const Matrix& leftVector, Matrix& ssnFunctionResult) {

	const Real V_a = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	const Real V_b = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	const Real V_c = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	const Real OMEGA = nonExplicitState(0, 0);

	const Real v_a = mPrevIntfVoltage(0, 0);
	const Real v_b = mPrevIntfVoltage(1, 0);
	const Real v_c = mPrevIntfVoltage(2, 0);
	const Real omega = State(3, 0);

	const Real X_d = mX_d;
	const Real X_dDash = mX_dDash;
	const Real X_q = mX_q;
	const Real X_qDash = mX_qDash;
	const Real H = mH;
	const Real T_d0Dash = mT_d0Dash;
	const Real T_q0Dash = mT_q0Dash;

	const Real e_dDash = State(1, 0);
	const Real e_qDash = State(2, 0);

	const Real e_f = me_f;
	const Real E_f = mE_f;

		mI_a = 	(cos(OMEGA)/X_dDash)*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
				+(cos(OMEGA)/X_dDash)*(((X_dDash*timeStep)/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(e_f+E_f))
				+(cos(OMEGA)/X_dDash)*((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(v_a*sin(omega)+v_b*sin(omega-(2.*PI/3.))+v_c*sin(omega+(2.*PI/3.)))
				+(cos(OMEGA)/X_dDash)*((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
				+(cos(OMEGA)/X_dDash)*((2.*T_d0Dash*X_dDash-timeStep*X_d)/(2.*T_d0Dash*X_dDash+timeStep*X_d))*e_qDash
				-(sin(OMEGA)/X_qDash)*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
				+(sin(OMEGA)/X_qDash)*((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(v_a*cos(omega)+v_b*cos(omega-(2.*PI/3.))+v_c*cos(omega+(2.*PI/3.)))
				+(sin(OMEGA)/X_qDash)*((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
				+(sin(OMEGA)/X_qDash)*((2.*T_q0Dash*X_qDash-timeStep*X_q)/(2.*T_q0Dash*X_qDash+timeStep*X_q))*e_dDash;

		mI_b = 	(cos(OMEGA-(2.*PI/3.))/X_dDash)*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
				+(cos(OMEGA-(2.*PI/3.))/X_dDash)*(((X_dDash*timeStep)/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(e_f+E_f))
				+(cos(OMEGA-(2.*PI/3.))/X_dDash)*((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(v_a*sin(omega)+v_b*sin(omega-(2.*PI/3.))+v_c*sin(omega+(2.*PI/3.)))
				+(cos(OMEGA-(2.*PI/3.))/X_dDash)*((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
				+(cos(OMEGA-(2.*PI/3.))/X_dDash)*((2.*T_d0Dash*X_dDash-timeStep*X_d)/(2.*T_d0Dash*X_dDash+timeStep*X_d))*e_qDash
				-(sin(OMEGA-(2.*PI/3.))/X_qDash)*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
				+(sin(OMEGA-(2.*PI/3.))/X_qDash)*((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(v_a*cos(omega)+v_b*cos(omega-(2.*PI/3.))+v_c*cos(omega+(2.*PI/3.)))
				+(sin(OMEGA-(2.*PI/3.))/X_qDash)*((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
				+(sin(OMEGA-(2.*PI/3.))/X_qDash)*((2.*T_q0Dash*X_qDash-timeStep*X_q)/(2.*T_q0Dash*X_qDash+timeStep*X_q))*e_dDash;

		mI_c = 	(cos(OMEGA+(2.*PI/3.))/X_dDash)*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
				+(cos(OMEGA+(2.*PI/3.))/X_dDash)*(((X_dDash*timeStep)/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(e_f+E_f))
				+(cos(OMEGA+(2.*PI/3.))/X_dDash)*((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(v_a*sin(omega)+v_b*sin(omega-(2.*PI/3.))+v_c*sin(omega+(2.*PI/3.)))
				+(cos(OMEGA+(2.*PI/3.))/X_dDash)*((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
				+(cos(OMEGA+(2.*PI/3.))/X_dDash)*((2.*T_d0Dash*X_dDash-timeStep*X_d)/(2.*T_d0Dash*X_dDash+timeStep*X_d))*e_qDash
				-(sin(OMEGA+(2.*PI/3.))/X_qDash)*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
				+(sin(OMEGA+(2.*PI/3.))/X_qDash)*((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(v_a*cos(omega)+v_b*cos(omega-(2.*PI/3.))+v_c*cos(omega+(2.*PI/3.)))
				+(sin(OMEGA+(2.*PI/3.))/X_qDash)*((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
				+(sin(OMEGA+(2.*PI/3.))/X_qDash)*((2.*T_q0Dash*X_qDash-timeStep*X_q)/(2.*T_q0Dash*X_qDash+timeStep*X_q))*e_dDash;

	Real F_omega = 	OMEGA-(-(pow(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)),2.))*timeStep/(4.*H*X_qDash)
					+(((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(v_a*cos(omega)+v_b*cos(omega-(2.*PI/3.))+v_c*cos(omega+(2.*PI/3.)))
					+((timeStep*(X_q-X_qDash))/(2.*T_q0Dash*X_qDash+X_q*timeStep))*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
					+((2.*T_q0Dash*X_qDash-timeStep*X_q)/(2.*T_q0Dash*X_qDash+timeStep*X_q))*e_dDash)*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))*timeStep/(4.*H*X_qDash)
					+(pow(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)), 2.))*timeStep/(4.*H*X_dDash)
					+(((X_dDash*timeStep)/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(e_f+E_f)
					+((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(v_a*sin(omega)+v_b*sin(omega-(2.*PI/3.))+v_c*sin(omega+(2.*PI/3.)))
					+((timeStep*(X_dDash-X_d))/(2.*T_d0Dash*X_dDash+X_d*timeStep))*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
					+((2.*T_d0Dash*X_dDash-timeStep*X_d)/(2.*T_d0Dash*X_dDash+timeStep*X_d))*e_qDash)*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))*timeStep/(4.*H*X_dDash));

	if (terminalNotGrounded(0)) {
		Math::setVectorElement(ssnFunctionResult, matrixNodeIndex(0, 0), -mI_a);
		Math::setVectorElement(ssnFunctionResult, matrixNodeIndex(0, 1), -mI_b);
		Math::setVectorElement(ssnFunctionResult, matrixNodeIndex(0, 2), -mI_c);
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(ssnFunctionResult, matrixNodeIndex(1, 0), mI_a);
		Math::setVectorElement(ssnFunctionResult, matrixNodeIndex(1, 1), mI_b);
		Math::setVectorElement(ssnFunctionResult, matrixNodeIndex(1, 2), mI_c);
	}
	Math::setVectorElement(ssnFunctionResult, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), F_omega);
}

void EMT::Ph3::SSN::SG4O::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(attribute("v_intf"));
	prevStepDependencies.push_back(attribute("i_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void EMT::Ph3::SSN::SG4O::stepInPerUnit()
{
	mnaApplyRightSideVectorStamp(**mRightVector);
	Matrix temp = Matrix::Zero(4,1);
	temp(0, 0) = (**mIntfVoltage)(0, 0);
	temp(1, 0) = (**mIntfVoltage)(1, 0);
	temp(2, 0) = (**mIntfVoltage)(2, 0);
	temp(3, 0) = nonExplicitState(0, 0);
	
	ssnUpdate(temp);
	//ssnUpdateNonExplicitStates(temp);
	//ssnUpdateJacobian(temp);
}

void EMT::Ph3::SSN::SG4O::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph3::SSN::SG4O::ssnUpdateNonExplicitStates(const Matrix& leftVector)
{
	nonExplicitState(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single));
}

void EMT::Ph3::SSN::SG4O::ssnUpdate(const Matrix& leftVector)
{
	ssnUpdateNonExplicitStates(leftVector);

	ssnUpdateJacobian(leftVector);
	ssnCalculateFunctionResult(leftVector, **SSN_Function_Result);
}

void EMT::Ph3::SSN::SG4O::ssnUpdateInputs(Real time)
{
	me_f = mE_f;
	mp_mech = mP_mech;
	/// TODO: Assign some new value to mP_mech, mE_f, maybe as a Signal...
}



void EMT::Ph3::SSN::SG4O::ssnUpdateState()
{
	const Real V_a = (**mIntfVoltage)(0, 0);
	const Real V_b = (**mIntfVoltage)(1, 0);
	const Real V_c = (**mIntfVoltage)(2, 0);
	const Real OMEGA = nonExplicitState(0, 0);

	const Real v_a = mPrevIntfVoltage(0, 0);
	const Real v_b = mPrevIntfVoltage(1, 0);
	const Real v_c = mPrevIntfVoltage(2, 0);
	const Real omega = State(3, 0);

	// delta
	State(0, 0) = State(0, 0) + timeStep*mOmega_base*(0.5*State(3, 0) - 1.) + 0.5*nonExplicitState(0, 0)*timeStep*mOmega_base;

	// E_d'
	State (1, 0) = 	((timeStep*(mX_q-mX_qDash))/(2.*mT_q0Dash*mX_qDash+mX_q*timeStep))*(v_a*cos(omega)+v_b*cos(omega-(2.*PI/3.))+v_c*cos(omega+(2.*PI/3.)))
					+((timeStep*(mX_q-mX_qDash))/(2.*mT_q0Dash*mX_qDash+mX_q*timeStep))*(V_a*cos(OMEGA)+V_b*cos(OMEGA-(2.*PI/3.))+V_c*cos(OMEGA+(2.*PI/3.)))
					+((2.*mT_q0Dash*mX_qDash-timeStep*mX_q)/(2.*mT_q0Dash*mX_qDash+timeStep*mX_q))*State(1, 0);

	// E_q'
	State(2, 0) = 	((mX_dDash*timeStep)/(2.*mT_d0Dash*mX_dDash+mX_d*timeStep))*(me_f+mE_f)
					+((timeStep*(mX_dDash-mX_d))/(2.*mT_d0Dash*mX_dDash+mX_d*timeStep))*(v_a*sin(omega)+v_b*sin(omega-(2.*PI/3.))+v_c*sin(omega+(2.*PI/3.)))
					+((timeStep*(mX_dDash-mX_d))/(2.*mT_d0Dash*mX_dDash+mX_d*timeStep))*(V_a*sin(OMEGA)+V_b*sin(OMEGA-(2.*PI/3.))+V_c*sin(OMEGA+(2.*PI/3.)))
					+((2.*mT_d0Dash*mX_dDash-timeStep*mX_d)/(2.*mT_d0Dash*mX_dDash+timeStep*mX_d))*State(2, 0);

	// OMEGA
	State(3, 0) = nonExplicitState(0, 0);
}

void EMT::Ph3::SSN::SG4O::mnaPostStep(const Matrix& leftVector) {
	mnaUpdateVoltage(leftVector);
	mnaUpdateCurrent(leftVector);
    ssnUpdateState();
}

void EMT::Ph3::SSN::SG4O::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mPrevIntfVoltage = **mIntfVoltage;
	**mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
		(**mIntfVoltage)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
		(**mIntfVoltage)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		(**mIntfVoltage)(1, 0) = (**mIntfVoltage)(1, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		(**mIntfVoltage)(2, 0) = (**mIntfVoltage)(2, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	}
	mSLog->debug(
		"\nUpdate Voltage: {:s}",
		Logger::matrixToString(**mIntfVoltage)
	);
}

void EMT::Ph3::SSN::SG4O::mnaUpdateCurrent(const Matrix& leftVector) {
    (**mIntfCurrent)(0,0) = mI_a;
	(**mIntfCurrent)(1,0) = mI_b;
	(**mIntfCurrent)(2,0) = mI_c;

	mSLog->debug(
		"\nUpdate Current: {:s}",
		Logger::matrixToString(**mIntfCurrent)
	);
	mSLog->flush();
}


void EMT::Ph3::SSN::SG4O::ssnUpdateJacobian(const Matrix& leftVector)
{
		/// TODO: Can exchange all local constants used in the equations by the member constants

		const Real V_a = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		const Real V_b = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		const Real V_c = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
		const Real OMEGA = nonExplicitState(0, 0);

		const Real v_a = mPrevIntfVoltage(0, 0);
		const Real v_b = mPrevIntfVoltage(1, 0);
		const Real v_c = mPrevIntfVoltage(2, 0);
		const Real omega = State(3, 0);

		const Real X_d = mX_d;
		const Real X_dDash = mX_dDash;
		const Real X_q = mX_q;
		const Real X_qDash = mX_qDash;
		const Real H = mH;
		const Real T_d0Dash = mT_d0Dash;
		const Real T_q0Dash = mT_q0Dash;

		const Real e_dDash = State(1, 0);
		const Real e_qDash = State(2, 0);

		const Real e_f = me_f;
		const Real E_f = mE_f;


		// 	dI_a/dV_a
		Jacobian(0, 0) =		cos(OMEGA)*sin(OMEGA)*	((timeStep*(X_d-X_qDash))/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
														+(timeStep*(X_dDash-X_d))/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
														-1./X_qDash
														+1./X_dDash);
		//	dI_a/dV_b
		Jacobian(0, 1) =		(sin(OMEGA)*cos(OMEGA-(2.*PI/3.))/X_qDash)*((((X_q-X_qDash)*timeStep)/(X_q*timeStep+2.*X_qDash*T_q0Dash))-1.)
								+(cos(OMEGA)*sin(OMEGA-(2.*PI/3.))/X_dDash)*((((X_dDash-X_d)*timeStep)/(X_d*timeStep+2.*X_dDash*T_d0Dash))+1.);
														
		//	dI_a/dV_c
		Jacobian(0, 2) =		(sin(OMEGA)*cos(OMEGA+(2.*PI/3.))/X_qDash)*((((X_q-X_qDash)*timeStep)/(X_q*timeStep+2.*X_qDash*T_q0Dash))-1.)
								+(cos(OMEGA)*sin(OMEGA+(2.*PI/3.))/X_dDash)*((((X_dDash-X_d)*timeStep)/(X_d*timeStep+2.*X_dDash*T_d0Dash))+1.);

		// 	dI_a/dOMEGA
		Jacobian(0, 3) =		sin(OMEGA)*(V_c*sin(OMEGA+(2.*PI/3.))+V_b*sin(OMEGA-(2.*PI/3.))+V_a*sin(OMEGA))
								*((X_d-X_dDash)*timeStep/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-1./X_dDash
								+(X_qDash-X_q)*timeStep/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+1./X_qDash)
								+cos(OMEGA)*(V_c*cos(OMEGA+(2.*PI/3.))+V_b*cos(OMEGA-(2.*PI/3.))+V_a*cos(OMEGA))
								*((X_q-X_qDash)*timeStep/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+(X_dDash-X_d)*timeStep/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-1./X_qDash
								+1./X_dDash)
								+sin(OMEGA)*(-(((X_dDash-X_d)*timeStep*(v_c*sin(omega+(2.*PI/3.))+v_b*sin(omega-(2.*PI/3.))+v_a*sin(omega)))/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash)))
								-((e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep))/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash)))
								-((timeStep*(e_f+E_f))/(X_d*timeStep+2.*X_dDash*T_d0Dash)))
								+(cos(OMEGA)/(X_qDash*(X_q*timeStep)+2.*X_qDash*T_q0Dash))
								*((X_q-X_qDash)*timeStep*(v_c*cos(omega+(2.*PI/3.))+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep));

		//	dI_b/dV_a
		Jacobian(1, 0) =		(cos(OMEGA)*sin(OMEGA-(2.*PI/3.))/X_qDash)*(((X_q-X_qDash)*timeStep)/(X_q*timeStep+2.*X_qDash*T_q0Dash)-1.)
								+(sin(OMEGA)*cos(OMEGA-(2.*PI/3.))/X_dDash)*((X_dDash-X_d)*timeStep/(X_d*timeStep+2.*X_dDash*T_d0Dash)+1.);
		//	dI_b/dV_b
		Jacobian(1, 1) =		cos(OMEGA-(2.*PI/3.))*sin(OMEGA-(2.*PI/3.))*((X_q-X_qDash)*timeStep/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+(X_dDash-X_d)*timeStep/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-1./X_qDash
								+1./X_dDash);
		//	dI_b/dV_c
		Jacobian(1, 2) =		(sin(OMEGA)*cos(OMEGA+(2.*PI/3.))/X_qDash)*((X_q-X_qDash)*timeStep/(X_q*timeStep+2.*X_qDash*T_q0Dash)-1)
								+(cos(OMEGA-2.*PI/3.)*sin(OMEGA+(2.*PI/3.))/(X_dDash))*((X_dDash-X_d)*timeStep/(X_d*timeStep+2.*X_dDash*T_d0Dash)+1.);

		//	dI_b/dOMEGA
		Jacobian(1, 3) =		(-sin(OMEGA-(2.*PI/3.))*(V_c*sin(OMEGA+(2*PI/3.))+V_b*sin(OMEGA-(2.*PI/3.))+V_a*sin(OMEGA)))
								*((1./X_dDash)*((X_dDash-X_d)*timeStep/(X_d*timeStep+2.*X_dDash*T_d0Dash)+1.)
								+(1./X_qDash)*((X_q-X_qDash)*timeStep/(X_q*timeStep+2.*X_qDash*T_q0Dash)-1.))
								+(cos(OMEGA-(2.*PI/3.))*(V_c*cos(OMEGA+(2*PI/3.))+V_b*cos(OMEGA-(2.*PI/3.))+V_a*cos(OMEGA)))
								*((1./X_qDash)*((X_q-X_qDash)*timeStep/(X_q*timeStep+2.*X_qDash*T_q0Dash)-1.)
								+(1./X_dDash)*((X_dDash-X_d)*timeStep/(X_d*timeStep+2.*X_dDash*T_d0Dash)+1.))
								-(X_dDash-X_d)*timeStep*(v_c*sin(omega+2.*PI/3.)+v_b*sin(omega-2.*PI/3.)+v_a*sin(omega))*sin(OMEGA-2.*PI/3.)/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep)*sin(OMEGA-2.*PI/3.)/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-(e_f+E_f)*timeStep*sin(OMEGA-2.*PI/3.)/(X_d*timeStep+2.*X_dDash*T_d0Dash)
								+(X_q-X_qDash)*timeStep*(v_c*cos(omega+2.*PI/3.)+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))*cos(OMEGA-2.*PI/3.)/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep)*cos(OMEGA-2.*PI/3.)/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash));

		//	dI_c/dV_a
		Jacobian(2, 0) =		(cos(OMEGA)*sin(OMEGA+(2.*PI/3.))/X_qDash)*(((X_q-X_qDash)*timeStep)/(X_q*timeStep+2.*X_qDash*T_q0Dash)-1.)
								+(sin(OMEGA)*cos(OMEGA+(2.*PI/3.))/X_dDash)*((X_dDash-X_d)*timeStep/(X_d*timeStep+2.*X_dDash*T_d0Dash)+1.);


		//	dI_c/dV_b
		Jacobian(2, 1) =		(cos(OMEGA-2.*PI/3.)*sin(OMEGA+(2.*PI/3.))/X_qDash)*((X_q-X_qDash)*timeStep/(X_q*timeStep+2.*X_qDash*T_q0Dash)-1)
								+(sin(OMEGA-2.*PI/3.)*cos(OMEGA+(2.*PI/3.))/(X_dDash))*((X_dDash-X_d)*timeStep/(X_d*timeStep+2.*X_dDash*T_d0Dash)+1.);

		//	dI_c/dV_c
		Jacobian(2, 2) =		cos(OMEGA+(2.*PI/3.))*sin(OMEGA+(2.*PI/3.))*((X_q-X_qDash)*timeStep/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+(X_dDash-X_d)*timeStep/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-1./X_qDash
								+1./X_dDash);

		//	dI_c/dOMEGA
		Jacobian(2, 3) =		sin(OMEGA+(2.*PI/3.))*(V_c*sin(OMEGA+(2.*PI/3.))+V_b*sin(OMEGA-(2.*PI/3.))+V_a*sin(OMEGA))
								*((X_d-X_dDash)*timeStep/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-1./X_dDash
								+(X_qDash-X_q)*timeStep/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+1./X_qDash)
								+cos(OMEGA+(2.*PI/3.))*(V_c*cos(OMEGA+(2.*PI/3.))+V_b*cos(OMEGA-(2.*PI/3.))+V_a*cos(OMEGA))
								*((X_q-X_qDash)*timeStep/(X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								+(X_dDash-X_d)*timeStep/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								-1./X_qDash
								+1./X_dDash)
								+sin(OMEGA+(2.*PI/3.))*(-(((X_dDash-X_d)*timeStep*(v_c*sin(omega+(2.*PI/3.))+v_b*sin(omega-(2.*PI/3.))+v_a*sin(omega)))/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash)))
								-((e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep))/(X_dDash*(X_d*timeStep+2.*X_dDash*T_d0Dash)))
								-((timeStep*(e_f+E_f))/(X_d*timeStep+2.*X_dDash*T_d0Dash)))
								+(cos(OMEGA+(2.*PI/3.))/(X_qDash*(X_q*timeStep)+2.*X_qDash*T_q0Dash))
								*((X_q-X_qDash)*timeStep*(v_c*cos(omega+(2.*PI/3.))+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep));

		//df_OMEGA/dV_a

		Jacobian(3, 0) =		-sin(OMEGA)*timeStep/(4.*X_dDash*H*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								*(2.*(X_dDash-X_d)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))
								+(X_dDash-X_d)*timeStep*(v_c*sin(omega+2.*PI/3.)+v_b*sin(omega-2.*PI/3.)+v_a*sin(omega))
								+e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep)
								+X_dDash*timeStep*(e_f+E_f))
								-sin(OMEGA)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))/(2.*X_dDash*H)
								+cos(OMEGA)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))/(2.*X_qDash*H)
								-cos(OMEGA)*timeStep/(4.*H*X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								*(2.*(X_q-X_qDash)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))
								+(X_q-X_qDash)*timeStep*(v_c*cos(omega+2.*PI/3.)+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))
								+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep));


		//df_OMEGA/dV_b

		Jacobian(3, 1) =		-sin(OMEGA-2.*PI/3.)*timeStep/(4.*X_dDash*H*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								*(2.*(X_dDash-X_d)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))
								+(X_dDash-X_d)*timeStep*(v_c*sin(omega+2.*PI/3.)+v_b*sin(omega-2.*PI/3.)+v_a*sin(omega))
								+e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep)
								+X_dDash*timeStep*(e_f+E_f))
								-sin(OMEGA-2.*PI/3.)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))/(2.*X_dDash*H)
								+cos(OMEGA-2.*PI/3.)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))/(2.*X_qDash*H)
								-cos(OMEGA-2.*PI/3.)*timeStep/(4.*H*X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								*(2.*(X_q-X_qDash)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))
								+(X_q-X_qDash)*timeStep*(v_c*cos(omega+2.*PI/3.)+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))
								+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep));

		//df_OMEGA/dV_c

		Jacobian(3, 2) =		-sin(OMEGA+2.*PI/3.)*timeStep/(4.*X_dDash*H*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								*(2.*(X_dDash-X_d)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))
								+(X_dDash-X_d)*timeStep*(v_c*sin(omega+2.*PI/3.)+v_b*sin(omega-2.*PI/3.)+v_a*sin(omega))
								+e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep)
								+X_dDash*timeStep*(e_f+E_f))
								-sin(OMEGA+2.*PI/3.)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))/(2.*X_dDash*H)
								+cos(OMEGA+2.*PI/3.)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))/(2.*X_qDash*H)
								-cos(OMEGA+2.*PI/3.)*timeStep/(4.*H*X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								*(2.*(X_q-X_qDash)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))
								+(X_q-X_qDash)*timeStep*(v_c*cos(omega+2.*PI/3.)+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))
								+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep));

		//df_OMEGA/dOMEGA

		Jacobian(3, 3) =	1.	-timeStep*(V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.)+V_a*cos(OMEGA))/(4.*X_dDash*H*(X_d*timeStep+2.*X_dDash*T_d0Dash))
								*(2.*(X_dDash-X_d)*timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))
								+(X_dDash-X_d)*timeStep*(v_c*sin(omega+2.*PI/3.)+v_b*sin(omega-2.*PI/3.)+v_a*sin(omega))
								+e_qDash*(2.*X_dDash*T_d0Dash-X_d*timeStep)
								+X_dDash*timeStep*(e_f+E_f))
								-timeStep*(V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.)+V_a*cos(OMEGA))*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))/(2.*X_dDash*H)
								-timeStep*(V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.)+V_a*cos(OMEGA))*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))/(2.*X_qDash*H)
								+timeStep*(sin(OMEGA)*V_a+V_c*sin(OMEGA+2.*PI/3.)+V_b*sin(OMEGA-2.*PI/3.))/(4.*H*X_qDash*(X_q*timeStep+2.*X_qDash*T_q0Dash))
								*(2.*(X_q-X_qDash)*timeStep*(cos(OMEGA)*V_a+V_c*cos(OMEGA+2.*PI/3.)+V_b*cos(OMEGA-2.*PI/3.))
								+(X_q-X_qDash)*timeStep*(v_c*cos(omega+2.*PI/3.)+v_b*cos(omega-2.*PI/3.)+v_a*cos(omega))
								+e_dDash*(2.*X_qDash*T_q0Dash-X_q*timeStep));		
}		