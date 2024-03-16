#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator4OrderSSN.h>

CPS::EMT::Ph3::SynchronGenerator4OrderSSN::SynchronGenerator4OrderSSN(String uid, String name, Logger::Level logLevel)
    	:Base::ReducedOrderSynchronGenerator<Real>(uid, name, logLevel)
{
    mPhaseType = PhaseType::ABC;
    setTerminalNumber(2);
    setVirtualNodeNumber(1);
    **mIntfCurrent = Matrix::Zero(3,1);
    **mIntfVoltage = Matrix::Zero(3,1);
}

CPS::EMT::Ph3::SynchronGenerator4OrderSSN::SynchronGenerator4OrderSSN(String name, Logger::Level logLevel)
	:SynchronGenerator4OrderSSN(name, name, logLevel)
{
}

CPS::EMT::Ph3::SynchronGenerator4OrderSSN::specificInitialization()
{
	(**mIntfCurrent)(0,0) = mInitCurrent.real;
	(**mIntfCurrent)(0,1) = (mInitCurrent * SHIFT_TO_PHASE_B).real;
	(**mIntfCurrent)(0,2) = (mInitCurrent * SHIFT_TO_PHASE_C).real;

	(**mIntfVoltage)(0,0) = mInitVoltage.real;
	(**mIntfVoltage)(0,1) = (mInitVoltage * SHIFT_TO_PHASE_B).real;
	(**mIntfVoltage)(0,2) = (mInitVoltage * SHIFT_TO_PHASE_C).real;

	P_mech = mInitMechPower;

	Vd = (**mIntfVoltage)(0,0)*cos(theta)+(**mIntfVoltage)(1,0)*cos(theta-(2.*M_PI/3.))+(**mIntfVoltage)(2,0)*cos(theta+(2.*M_PI/3.));
    Vq = -((**mIntfVoltage)(0,0)*sin(theta)+(**mIntfVoltage)(1,0)*sin(theta-(2.*M_PI/3.))+(**mIntfVoltage)(2,0)*sin(theta+(2.*M_PI/3.)));

	updateCurrentStates();

	double C_d = (mTimeStep*mLd_t)/(2.*mTd0_t*mLd_t+mTimeStep*mLd);
    double C_dd = (mTimeStep*(mLd-mLd_t))/(2.*mTd0_t*mLd_t+mTimeStep*mLd);
    double C_0dd = (2.*mTd0_t*mLd_t-mTimeStep*mLd)/(2.*mTd0_t*mLd_t+mTimeStep*mLd);
    double C_qq = (mTimeStep*(mLq-mLq_t))/(2.*mTq0_t*mLq_t+mTimeStep*mLq);
    double C_0qq = (2.*mTq0_t*mLq_t-mTimeStep*mLq)/(2.*mTq0_t*mLq_t+mTimeStep*mLq);
    double C_wbq = (mTimeStep*mTimeStep*mBase_OmElec)/(4.*mH*mLq_t);
    double C_wbd = (mTimeStep*mTimeStep*mBase_OmElec)/(4.*mH*mLd_t);
    double C_wb = (mTimeStep*mTimeStep*mBase_OmElec)/(8.*mH);
    double C_h = (mTimeStep)/(4.*mH);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::calculateNonlinearFunctionResult()
{
    (**mIntfCurrent)(0,0) = ((Eq-Vq)/mLd)*cos(theta) - ((Vd-Ed)/mLq)*sin(theta);
    (**mIntfCurrent)(1,0) = ((Eq-Vq)/mLd)*cos(theta - (2*M_PI/3)) - ((Vd-Ed)/mLq)*sin(theta - (2*M_PI/3));
    (**mIntfCurrent)(2,0) = ((Eq-Vq)/mLd)*cos(theta + (2*M_PI/3)) - ((Vd-Ed)/mLq)*sin(theta + (2*M_PI/3));

	double f_theta = C_wb*P_mech - C_wb*((Vd*Vd)/mLq_t)
			+C_wb*((Vd)/mLq_t)*(C_qq*Vd+C_qq*Vd_old+C_0qq*Ed_old)
			-C_wb*(Vq/mLd_t)*(C_dd*Vq+C_d*Ef+C_0dd*Eq_old+C_dd*Vq_old+C_d*Ef_old)
			+C_wb*((Vq*Vq)/mLd_t)
			-theta;

    if (terminalNotGrounded(0)) {
		Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(0, 0), -(**mIntfCurrent)(0,0));
		Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(0, 1), -(**mIntfCurrent)(1,0));
		Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(0, 2), -(**mIntfCurrent)(2,0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(1, 0), (**mIntfCurrent)(0,0));
		Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(1, 1), (**mIntfCurrent)(1,0));
		Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(1, 2), (**mIntfCurrent)(2,0));
	}
	Math::setVectorElement(**mNonlinearFunctionStamp, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), f_theta);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), Jacobian(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(0, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), Jacobian(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(1, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), Jacobian(2, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), -Jacobian(2, 3));
	}
	if (terminalNotGrounded(1)) {
		// set bottom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), Jacobian(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(0, 3));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), Jacobian(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(1, 3));
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
		//Internal Voltages are v1-v0: Positive for Terminal 1, negative for terminal 0
        //Theta is not a difference: We only have one "virtual terminal" for it.

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(0, 0) ,-Jacobian(3, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(0, 1) ,-Jacobian(3, 1));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(0, 2) ,-Jacobian(3, 2));

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(1, 0) ,Jacobian(3, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(1, 1) ,Jacobian(3, 1));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), matrixNodeIndex(1, 2) ,Jacobian(3, 2));

		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), Jacobian(3, 3));

		//Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1.);
		//Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1.);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	//Phase current output equations do not contain constant history terms

	//Math::setVectorElement(rightVector, matrixNodeIndex(0, PhaseType::A), 0.);
	//Math::setVectorElement(rightVector, matrixNodeIndex(0, PhaseType::B), 0.);
	//Math::setVectorElement(rightVector, matrixNodeIndex(0, PhaseType::C), 0.);

	//Math::setVectorElement(rightVector, matrixNodeIndex(0, PhaseType::A), 0.);
	//Math::setVectorElement(rightVector, matrixNodeIndex(0, PhaseType::B), 0.);
	//Math::setVectorElement(rightVector, matrixNodeIndex(0, PhaseType::C), 0.);

	//Equation for theta does contain constant history term:

	double linear_theta_hist = -C_wb*(P_mech_old-(Vd_old*Vd_old/mLq_t)+(Vd_old*Ed_old/mLq_t)-(Vq_old*Eq_old/mLd_t)+(Vq*Vq/mLd_t))-(0.5*mTimeStep*mBase_OmElec*omega_old)-(0.5*mTimeStep*mBase_OmElec*(omega_old-2.))-omega_old-1.;
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single), linear_theta_hist);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
}


void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompPostStep(const Matrix &leftVector) {
    updateOldStates();
	mnaCompUpdateVoltage(leftVector);
	mnaCompUpdateCurrent(leftVector);
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	(**mIntfVoltage) = Matrix::Zero(3,1);
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1,0));
		(**mIntfVoltage)(1,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1,1));
		(**mIntfVoltage)(2,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1,2));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0,0));
        (**mIntfVoltage)(1,0) = (**mIntfVoltage)(1,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0,1));
        (**mIntfVoltage)(2,0) = (**mIntfVoltage)(2,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0,2));

    Vd = (**mIntfVoltage)(0,0)*cos(theta)+(**mIntfVoltage)(1,0)*cos(theta-(2.*M_PI/3.))+(**mIntfVoltage)(2,0)*cos(theta+(2.*M_PI/3.));
    Vq = -((**mIntfVoltage)(0,0)*sin(theta)+(**mIntfVoltage)(1,0)*sin(theta-(2.*M_PI/3.))+(**mIntfVoltage)(2,0)*sin(theta+(2.*M_PI/3.)));
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::mnaCompUpdateCurrent(const Matrix& leftVector) {
	///FIXME: Is done every iteration anyways; do we need it here?
	calculateNonlinearFunctionResult(); 
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::iterationUpdate(const Matrix& leftVector)
{
    mnaCompUpdateVoltage(leftVector);
	updateImplicitStates(leftVector);
	updateCurrentStates();
	calculateNonlinearFunctionResult();
	updateJacobian();
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::updateJacobian()
{
	double V_a = (**mIntfVoltage)(0,0);
	double V_b = (**mIntfVoltage)(1,0);
	double V_c = (**mIntfVoltage)(2,0);

	Jacobian(0,0)=(cos(theta)*sin(theta)*(1.-C_dd)/mLd_t)-(cos(theta)*sin(theta)*(1.-C_qq)/mLq_t);
	Jacobian(0,1)=(cos(theta)*sin(theta-(2.*M_PI/3.))*(1.-C_dd)/mLd_t)-(cos(theta-(2.*M_PI/3.))*sin(theta)*(1.-C_qq)/mLq_t);
	Jacobian(0,2)=(cos(theta)*sin(theta+(2.*M_PI/3.))*(1.-C_dd)/mLd_t)-(cos(theta+(2.*M_PI/3.))*sin(theta)*(1.-C_qq)/mLq_t);
	Jacobian(0,3)=(cos(theta)/mLd_t)*(V_a*cos(theta)+V_b*cos(theta-(2.*M_PI/3.))+V_c*cos(theta+(2.*M_PI/3.)))*(1.-C_dd)-(sin(theta)/mLd_t)*(Eq-Vq)+(sin(theta)/mLq_t)*(V_a*sin(theta)+V_b*sin(theta-(2.*M_PI/3.))+V_c*sin(theta+(2.*M_PI/3.)))*(1.-C_qq)+(cos(theta)/mLq_t)*(Ed-Vd);
	Jacobian(1,0)=(cos(theta-(2.*M_PI/3.))*sin(theta)*(1.-C_dd)/mLd_t)-(cos(theta)*sin(theta-(2.*M_PI/3.))*(1.-C_qq)/mLq_t);
	Jacobian(1,1)=(cos(theta-(2.*M_PI/3.))*sin(theta-(2.*M_PI/3.))*(1.-C_dd)/mLd_t)-(cos(theta-(2.*M_PI/3.))*sin(theta-(2.*M_PI/3.))*(1.-C_qq)/mLq_t);
	Jacobian(1,2)=(cos(theta-(2.*M_PI/3.))*sin(theta+(2.*M_PI/3.))*(1.-C_dd)/mLd_t)-(cos(theta+(2.*M_PI/3.))*sin(theta-(2.*M_PI/3.))*(1.-C_qq)/mLq_t);
	Jacobian(1,3)=(cos(theta-(2.*M_PI/3.))/mLd_t)*(V_a*cos(theta)+V_b*cos(theta-(2.*M_PI/3.))+V_c*cos(theta+(2.*M_PI/3.)))*(1.-C_dd)-(sin(theta-(2.*M_PI/3.))/mLd_t)*(Eq-Vq)+(sin(theta-(2.*M_PI/3.))/mLq_t)*(V_a*sin(theta)+V_b*sin(theta-(2.*M_PI/3.))+V_c*sin(theta+(2.*M_PI/3.)))*(1.-C_qq)+(cos(theta-(2.*M_PI/3.))/mLq_t)*(Ed-Vd);
	Jacobian(2,0)=(cos(theta+(2.*M_PI/3.))*sin(theta)*(1.-C_dd)/mLd_t)-(cos(theta)*sin(theta+(2.*M_PI/3.))*(1.-C_qq)/mLq_t);
	Jacobian(2,1)=(cos(theta+(2.*M_PI/3.))*sin(theta-(2.*M_PI/3.))*(1.-C_dd)/mLd_t)-(cos(theta-(2.*M_PI/3.))*sin(theta+(2.*M_PI/3.))*(1.-C_qq)/mLq_t);
	Jacobian(2,2)=(cos(theta+(2.*M_PI/3.))*sin(theta+(2.*M_PI/3.))*(1.-C_dd)/mLd_t)-(cos(theta+(2.*M_PI/3.))*sin(theta+(2.*M_PI/3.))*(1.-C_qq)/mLq_t);
	Jacobian(2,3)=(cos(theta+(2.*M_PI/3.))/mLd_t)*(V_a*cos(theta)+V_b*cos(theta-(2.*M_PI/3.))+V_c*cos(theta+(2.*M_PI/3.)))*(1.-C_dd)-(sin(theta+(2.*M_PI/3.))/mLd_t)*(Eq-Vq)+(sin(theta+(2.*M_PI/3.))/mLq_t)*(V_a*sin(theta)+V_b*sin(theta-(2.*M_PI/3.))+V_c*sin(theta+(2.*M_PI/3.)))*(1.-C_qq)+(cos(theta+(2.*M_PI/3.))/mLq_t)*(Ed-Vd);
	Jacobian(3,0)=C_wbq*Vd*cos(theta)*(1.-C_qq)-0.5*C_wbq*cos(theta)*(C_qq*Vd_old+C_0qq*Ed_old)-C_wbd*Vq*sin(theta)*(C_dd-1.)-0.5*C_wbd*sin(theta)*(C_d*(Ef+Ef_old)+C_0dd*Eq_old+C_dd*Vq_old);
	Jacobian(3,1)=C_wbq*Vd*cos(theta-(2.*M_PI/3.))*(1.-C_qq)-0.5*C_wbq*cos(theta-(2.*M_PI/3.))*(C_qq*Vd_old+C_0qq*Ed_old)-C_wbd*Vq*sin(theta-(2.*M_PI/3.))*(C_dd-1.)-0.5*C_wbd*sin(theta-(2.*M_PI/3.))*(C_d*(Ef+Ef_old)+C_0dd*Eq_old+C_dd*Vq_old);
	Jacobian(3,2)=C_wbq*Vd*cos(theta+(2.*M_PI/3.))*(1.-C_qq)-0.5*C_wbq*cos(theta+(2.*M_PI/3.))*(C_qq*Vd_old+C_0qq*Ed_old)-C_wbd*Vq*sin(theta+(2.*M_PI/3.))*(C_dd-1.)-0.5*C_wbd*sin(theta+(2.*M_PI/3.))*(C_d*(Ef+Ef_old)+C_0dd*Eq_old+C_dd*Vq_old);
	Jacobian(3,3)=C_wbq*Vd*(V_a*sin(theta)+V_b*sin(theta-(2.*M_PI/3.))+V_c*sin(theta+(2.*M_PI/3.)))*(1.-C_qq)-0.5*C_wbq*(V_a*sin(theta)+V_b*sin(theta-(2.*M_PI/3.))+V_c*sin(theta+(2.*M_PI/3.)))*(C_qq*Vd_old+C_0qq*Ed_old)+C_wbd*Vq*(V_a*cos(theta)+V_b*cos(theta-(2.*M_PI/3.))+V_c*cos(theta+(2.*M_PI/3.)))*(C_dd-1.)+0.5*C_wbd*(V_a*cos(theta)+V_b*cos(theta-(2.*M_PI/3.))+V_c*cos(theta+(2.*M_PI/3.)))*(C_d*(Ef+Ef_old)+C_0dd*Eq_old+C_dd*Vq_old)-1.;
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::updateCurrentStates()
{
	Ed		= C_qq*(Vd+Vd_old)+C_0qq*Ed_old;
	Eq		= C_dd*(Vq+Vq_old)+C_d*(Ef+Ef_old)+C_0dd*Eq_old;
	omega	= C_h*(P_mech-(Vd*Vd/mLq_t)+(Vd*Ed/mLq_t)-(Vq*Eq/mLd_t)+(Vq*Vq/mLd_t))+C_h*(P_mech_old-(Vd_old*Vd_old/mLq_t)+(Vd_old*Ed_old/mLq_t)-(Vq_old*Eq_old/mLd_t)+(Vq_old*Vq_old/mLd_t))+omega_old;
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::updateOldStates()
{
	Ed_old = Ed;
	Eq_old = Eq;
	omega_old = omega;
	theta_old = theta;
	Vd_old = Vd;
	Vq_old = Vq;
}

void CPS::EMT::Ph3::SynchronGenerator4OrderSSN::updateImplicitStates(const Matrix& leftVector)
{
	theta = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::Single));
}
