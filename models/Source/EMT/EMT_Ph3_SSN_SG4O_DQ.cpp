#include <cps/EMT/EMT_Ph3_SSN_SG4O_DQ.h>

using namespace CPS;

EMT::Ph3::SSN::SG4O_DQ::SG4O_DQ(String uid, String name, Logger::Level logLevel):
    Base::ReducedOrderSynchronGenerator<Real>(uid, name, logLevel)
    {
        mPhaseType=PhaseType::ABC;
        setTerminalNumber(2);
        setVirtualNodeNumber(1);
    }


SimPowerComp<Real>::Ptr EMT::Ph3::SSN::SG4O_DQ::clone(String name){
    auto copy = SG4O_DQ::make(name, mLogLevel);
	return copy;
}

void EMT::Ph3::SSN::SG4O_DQ::specificInitialization(){

    (**mIntfVoltage)(0, 0) = (**mVdq0)(0, 0)*cos(**mOmMech) - (**mVdq0)(1, 0)*sin(**mOmMech);
    (**mIntfVoltage)(1, 0) = (**mVdq0)(0, 0)*cos(**mOmMech-2.*PI/3.) - (**mVdq0)(1, 0)*sin(**mOmMech-2.*PI/3.);
    (**mIntfVoltage)(2, 0) = (**mVdq0)(0, 0)*cos(**mOmMech+2.*PI/3.) - (**mVdq0)(1, 0)*sin(**mOmMech+2.*PI/3.);

    z(0, 0) = **mOmMech;

    **SSN_Function_Result = Matrix::Zero(mRightVector->get().rows(), 1);
    mMechPow = mInitMechPower/2.;
    mPrevMechPow = mInitMechPower/2.;

    SSNupdateStates();
    SSNcalculateFunctionResult();
    SSNupdateJacobian();
}

void EMT::Ph3::SSN::SG4O_DQ::stepInPerUnit(){
    //SSNupdateInputs();
}

void EMT::Ph3::SSN::SG4O_DQ::SSNupdateStates(){

    //States(0, 0): delta
    States(0, 0) =  prevStates(0, 0) + 0.5*mTimeStep*mBase_OmMech*(z(0, 0) + prev_z(0, 0))-mTimeStep*mBase_OmMech;

    //States(0, 0): Ed'
    States(1, 0) =  prevStates(1, 0)*(2.*mTq0_t*mLq_t-mLq*mTimeStep)/(2.*mTq0_t*mLq_t+mLq*mTimeStep)+prev_mVdq0(0, 0)*mTimeStep*(mLq-mLq_t)/(2.*mTq0_t*mLq_t+mLq*mTimeStep)
                    +((**mVdq0)(0, 0)*(mLq-mLq_t)*mTimeStep)/(2.*mTq0_t*mLq_t+mLq*mTimeStep);

    //States(0, 0): Eq'
    States(2, 0) =  prevStates(2, 0)*(2.*mTd0_t*mLd_t-mLd*mTimeStep)/(2.*mTd0_t*mLd_t+mLd*mTimeStep)+prev_mVdq0(1, 0)*mTimeStep*(mLd-mLd_t)/(2.*mTd0_t*mLd_t+mLd*mTimeStep)
                    +(mTimeStep*mLd_t*(mEf+prev_mEf))/(2.*mTd0_t*mLd_t+mLd*mTimeStep)+((**mVdq0)(1, 0)*(mLd-mLd_t)*mTimeStep)/(2.*mTd0_t*mLd_t+mLd*mTimeStep);
}

void EMT::Ph3::SSN::SG4O_DQ::SSNcalculateFunctionResult(){

    Real f_Vd       = -(**mVdq0)(0, 0)   + (**mIntfVoltage)(0, 0)*cos(z(0, 0))
                                        + (**mIntfVoltage)(1, 0)*cos(z(0, 0) - (2.*PI/3.))
                                        + (**mIntfVoltage)(2, 0)*cos(z(0, 0) + (2.*PI/3.));
    Real f_Vq       = -(**mVdq0)(1, 0)   - (**mIntfVoltage)(0, 0)*sin(z(0, 0))
                                        - (**mIntfVoltage)(1, 0)*sin(z(0, 0) - (2.*PI/3.))
                                        - (**mIntfVoltage)(2, 0)*sin(z(0, 0) + (2.*PI/3.));
    Real f_omega    = -z(0, 0)  + ((**mVdq0)(0, 0)*(**mVdq0)(0, 0)*mTimeStep/(4.*mH*mLq_t))*((mLq-mLq_t)*mTimeStep/(2.*mTq0_t*mLq_t+mLq*mTimeStep)-1.)
                                + ((**mVdq0)(1, 0)*(**mVdq0)(1, 0)*mTimeStep/(4.*mH*mLd_t))*(1.-(mLd-mLd_t)*mTimeStep/(2.*mTd0_t*mLd_t+mLd*mTimeStep))
                                +((mTimeStep*(**mVdq0)(0, 0))/(8.*mH*mTq0_t*mLq_t*mLq_t+4.*mH*mLq*mLq_t*mTimeStep))
                                *((2.*mTq0_t*mLq_t-mLq*mTimeStep)*prevStates(1, 0)+mTimeStep*(mLq-mLq_t)*prev_mVdq0(0, 0))
                                -((mTimeStep*(**mVdq0)(1, 0))/(8.*mH*mTd0_t*mLd_t*mLd_t+4.*mH*mLd*mLd_t*mTimeStep))
                                *((2.*mTd0_t*mLd_t-mLd*mTimeStep)*prevStates(2, 0)+mTimeStep*(mLd-mLd_t)*prev_mVdq0(1, 0)+mTimeStep*mLd_t*(mEf+prev_mEf));

    Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(1, 0), (**mIntfCurrent)(0, 0));
    Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(1, 1), (**mIntfCurrent)(1, 0));
    Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(1, 2), (**mIntfCurrent)(2, 0));
    
    Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(0, 0), -(**mIntfCurrent)(0, 0));
    Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(0, 1), -(**mIntfCurrent)(1, 0));
    Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(0, 2), -(**mIntfCurrent)(2, 0));

    Math::setVectorElement(**SSN_Function_Result, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), f_Vd);
    Math::setVectorElement(**SSN_Function_Result, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), f_Vq);
    Math::setVectorElement(**SSN_Function_Result, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), f_omega);
}


void EMT::Ph3::SSN::SG4O_DQ::ssnUpdate(const Matrix& leftVector){

    //Update phase voltages
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

    //Update Vd and Vq
	(**mVdq0)(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
	(**mVdq0)(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));

    //Update non-explicit states z
    z(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));

    SSNupdateStates();

    //update output phase currents
    (**mIntfCurrent)(0, 0) = (States(2, 0) - (**mVdq0)(1, 0))*(cos(z(0, 0))/mLd_t)-((**mVdq0)(0, 0) - States(1, 0))*(sin(z(0, 0))/mLq_t);
    (**mIntfCurrent)(1, 0) = (States(2, 0) - (**mVdq0)(1, 0))*(cos(z(0, 0) - (2.*PI/3.))/mLd_t)-((**mVdq0)(0, 0) - States(1, 0))*(sin(z(0, 0)- (2.*PI/3.))/mLq_t);
    (**mIntfCurrent)(2, 0) = (States(2, 0) - (**mVdq0)(1, 0))*(cos(z(0, 0) + (2.*PI/3.))/mLd_t)-((**mVdq0)(0, 0) - States(1, 0))*(sin(z(0, 0)+ (2.*PI/3.))/mLq_t);

    SSNcalculateFunctionResult();

    SSNupdateJacobian();
}


void EMT::Ph3::SSN::SG4O_DQ::SSNupdateJacobian(){

//------------------------------------------------------I_a-----------------------------------------------------------//
    
    Jacobian(0, 0) =    0.;  
    Jacobian(0, 1) =    0.;
    Jacobian(0, 2) =    0.;

    Jacobian(0, 3) =    -(sin(z(0, 0)/mLq_t))*(1.-((mLq-mLq_t)*mTimeStep)/(2.*mTq0_t*mLq_t+mLq*mTimeStep));    

    Jacobian(0, 4) =    (cos(z(0, 0))/mLd_t)*((mTimeStep*(mLd-mLd_t))/(2.*mTd0_t*mLd_t+mLd*mTimeStep)-1.);

    Jacobian(0, 5) =    -(sin(z(0, 0))/mLd_t)*(States(2, 0)-(**mVdq0)(1, 0))
                        -(cos(z(0, 0))/mLq_t)*((**mVdq0)(0, 0)-States(1, 0));            

//-------------------------------------------------------I_b----------------------------------------------------------//

    Jacobian(1, 0) =    0.;  
    Jacobian(1, 1) =    0.;
    Jacobian(1, 2) =    0.;

    Jacobian(1, 3) =    -(sin(z(0, 0)- (2.*PI/3.))/mLq_t)*(1.-((mLq-mLq_t)*mTimeStep)/(2.*mTq0_t*mLq_t+mLq*mTimeStep));    

    Jacobian(1, 4) =    (cos(z(0, 0) - (2.*PI/3.))/mLd_t)*((mTimeStep*(mLd-mLd_t))/(2.*mTd0_t*mLd_t+mLd*mTimeStep)-1.);

    Jacobian(1, 5) =    -(sin(z(0, 0)- (2.*PI/3.))/mLd_t)*(States(2, 0)-(**mVdq0)(1, 0))
                        -(cos(z(0, 0)- (2.*PI/3.))/mLq_t)*((**mVdq0)(0, 0)-States(1, 0));   

//-------------------------------------------------------I_c----------------------------------------------------------//

    Jacobian(2, 0) =    0.;  
    Jacobian(2, 1) =    0.;
    Jacobian(2, 2) =    0.;

    Jacobian(2, 3) =    -(sin(z(0, 0)+ (2.*PI/3.))/mLq_t)*(1.-((mLq-mLq_t)*mTimeStep)/(2.*mTq0_t*mLq_t+mLq*mTimeStep));    

    Jacobian(2, 4) =    (cos(z(0, 0) + (2.*PI/3.))/mLd_t)*((mTimeStep*(mLd-mLd_t))/(2.*mTd0_t*mLd_t+mLd*mTimeStep)-1.);

    Jacobian(2, 5) =    -(sin(z(0, 0)+ (2.*PI/3.))/mLd_t)*(States(2, 0)-(**mVdq0)(1, 0))
                        -(cos(z(0, 0)+ (2.*PI/3.))/mLq_t)*((**mVdq0)(0, 0)-States(1, 0));   


//-------------------------------------------------------f_Vd----------------------------------------------------------//

    Jacobian(3, 0) =    cos(z(0, 0));
    Jacobian(3, 1) =    cos(z(0, 0) - (2.*PI/3.));
    Jacobian(3, 2) =    cos(z(0, 0) + (2.*PI/3.));
    Jacobian(3, 3) =    -1.;
    Jacobian(3, 4) =    0.;
    Jacobian(3, 5) =    -sin(z(0, 0))*((**mIntfVoltage)(0, 0)) 
                        -sin(z(0, 0) - (2.*PI/3.))*((**mIntfVoltage)(1, 0))
                        -sin(z(0, 0) + (2.*PI/3.))*((**mIntfVoltage)(2, 0));

//-------------------------------------------------------f_Vq----------------------------------------------------------//

    Jacobian(4, 0) =    -sin(z(0, 0));
    Jacobian(4, 1) =    -sin(z(0, 0) - (2.*PI/3.));
    Jacobian(4, 2) =    -sin(z(0, 0) + (2.*PI/3.));
    Jacobian(4, 3) =    0.;
    Jacobian(4, 4) =    -1.;
    Jacobian(4, 5) =    -cos(z(0, 0))*((**mIntfVoltage)(0, 0)) 
                        -cos(z(0, 0) - (2.*PI/3.))*((**mIntfVoltage)(1, 0))
                        -cos(z(0, 0) + (2.*PI/3.))*((**mIntfVoltage)(2, 0));

//-------------------------------------------------------f_z(f_omega)----------------------------------------------------------//

    Jacobian(5, 0) =    0.;
    Jacobian(5, 1) =    0.;
    Jacobian(5, 2) =    0.;
    Jacobian(5, 3) =    (((**mVdq0)(0, 0)*mTimeStep)/(2.*mH*mLq_t))*(((mLq-mLq_t)*mTimeStep)/(2.*mTq0_t*mLq_t+mLq*mTimeStep)-1.)
                        +mTimeStep*(((2.*mTq0_t*mLq_t-mLq*mTimeStep)*prevStates(1, 0)+prev_mVdq0(0, 0)*mTimeStep*(mLq-mLq_t))
                        /(8.*mH*mTq0_t*mLq_t*mLq_t+4.*mH*mLq*mLq_t*mTimeStep));
    Jacobian(5, 4) =    (((**mVdq0)(1, 0)*mTimeStep)/(2.*mH*mLd_t))*(1.-((mLd-mLd_t)*mTimeStep)/(2.*mTd0_t*mLd_t+mLd*mTimeStep))
                        -mTimeStep*(((2.*mTd0_t*mLd_t-mLd*mTimeStep)*prevStates(2, 0)+prev_mVdq0(1, 0)*mTimeStep*(mLd-mLd_t)+mTimeStep*mLd_t*(mEf+prev_mEf))
                        /(8.*mH*mTd0_t*mLd_t*mLd_t+4.*mH*mLd*mLd_t*mTimeStep));
    Jacobian(5, 5) =    -1.;
}

void EMT::Ph3::SSN::SG4O_DQ::mnaApplySystemMatrixStamp(Matrix& systemMatrix){


if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), Jacobian(0, 2));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(0, 3));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(0, 4));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(0, 5));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), Jacobian(1, 2));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(1, 3));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(1, 4));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(1, 5));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), Jacobian(2, 2));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(2, 3));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(2, 4));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(2, 5));

        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), -Jacobian(3, 0));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 1), -Jacobian(4, 1));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 2), -Jacobian(5, 2));
        
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 0), -Jacobian(3, 0));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), -Jacobian(4, 1));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 2), -Jacobian(5, 2));

        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 0), -Jacobian(3, 0));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 1), -Jacobian(4, 1));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), -Jacobian(5, 2)); 
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), Jacobian(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), Jacobian(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), Jacobian(0, 2));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(0, 3));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(0, 4));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(0, 5));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), Jacobian(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), Jacobian(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), Jacobian(1, 2));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(1, 3));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(1, 4));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(1, 5));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), Jacobian(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), Jacobian(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), Jacobian(2, 2));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(2, 3));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(2, 4));
        Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(2, 5));

        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 0), Jacobian(3, 0));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 1), Jacobian(4, 1));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 2), Jacobian(5, 2));
        
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 0), Jacobian(3, 0));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 1), Jacobian(4, 1));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 2), Jacobian(5, 2));

        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 0), Jacobian(3, 0));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 1), Jacobian(4, 1));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 2), Jacobian(5, 2));      
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
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(3, 3));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(3, 4));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(3, 5));

        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(4, 3));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(4, 4));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(4, 5));

        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), Jacobian(5, 3));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), Jacobian(5, 4));
        Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), Jacobian(5, 5));
}

void EMT::Ph3::SSN::SG4O_DQ::mnaApplyRightSideVectorStamp(Matrix& rightVector){
    
    Real temp_omega =   -prev_z(0, 0)
                        -mTimeStep*(mMechPow+mPrevMechPow)/(4.*mH)
                        +mTimeStep*prev_mVdq0(0, 0)*(prev_mVdq0(0, 0) - prevStates(1, 0))/(4.*mH*mLq_t)
                        +mTimeStep*prev_mVdq0(1, 0)*(prevStates(2, 0) - prev_mVdq0(1, 0))/(4.*mH*mLd_t);

    zHistory(0, 0) = temp_omega;
    /*
    //All yHistory entries are zero
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

    //Vd and Vq do not have history terms, only the function for omega
    Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), zHistory(0, 0));
}


void EMT::Ph3::SSN::SG4O_DQ::mnaPostStep(const Matrix& leftVector){
    //Update all old quantities
    prevStates = States;
    prev_z = z;
    prev_mEf = mEf;
    mPrevMechPow = mMechPow;
    prev_mVdq0 = (**mVdq0);
}