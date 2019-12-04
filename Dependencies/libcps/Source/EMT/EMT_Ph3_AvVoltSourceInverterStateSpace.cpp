/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.h>

using namespace CPS;

EMT::Ph3::AvVoltSourceInverterStateSpace::AvVoltSourceInverterStateSpace(String uid, String name, Logger::Level logLevel)
: PowerComponent<Real>(uid, name, logLevel){
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Matrix>("V_cabc", &mVcabc, Flags::read | Flags::write);
	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read | Flags::write);
	addAttribute<Real>("p", &mP, Flags::read | Flags::write);
	addAttribute<Real>("q", &mQ, Flags::read | Flags::write);
	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q_ref", &mQref, Flags::read | Flags::write);
	addAttribute<Real>("theta", &mThetaPLL, Flags::read | Flags::write);
	addAttribute<Real>("phipll", &mPhiPLL, Flags::read | Flags::write);
	addAttribute<Real>("phid", &mPhi_d, Flags::read | Flags::write);
	addAttribute<Real>("phiq", &mPhi_q, Flags::read | Flags::write);
	addAttribute<Real>("gammad", &mGamma_d, Flags::read | Flags::write);
	addAttribute<Real>("gammaq", &mGamma_q, Flags::read | Flags::write);
}


void EMT::Ph3::AvVoltSourceInverterStateSpace::setParameters(Real sysOmega, Complex sysVoltNom, Real Pref, Real Qref, Real Lf, Real Cf,
	Real Rf, Real Rc, Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl) {

	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;
	mYc = 1. / Rc;

	mPref = Pref;
	mQref = Qref;

	mKpPLL = Kp_pll;
	mKiPLL = Ki_pll;

	mKiPowerCtrld = Ki_powerCtrl;
	mKiPowerCtrlq = Ki_powerCtrl;

	mKpPowerCtrld = Kp_powerCtrl;
	mKpPowerCtrlq = Kp_powerCtrl;

	mKiCurrCtrld = Ki_currCtrl;
	mKiCurrCtrlq = Ki_currCtrl;

	mKpCurrCtrld = Kp_currCtrl;
	mKpCurrCtrlq = Kp_currCtrl;

	// init with nodes at nominal voltage and branches have zero current flow.
	Real srcFreq = 2. * M_PI * sysOmega;
	Base::Ph1::VoltageSource::setParameters(sysVoltNom, srcFreq);
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::initializeStates(Real omega, Real timeStep,
	Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaN = omega;
	mOmegaCutoff = omega;
	// create matrices for state space representation
	mA = Matrix::Zero(14, 14);
	mB = Matrix::Zero(14, 6);
	mC = Matrix::Zero(3, 14);
	mD = Matrix::Zero(3, 6);
	mStates = Matrix::Zero(14, 1);
	mU = Matrix::Zero(6, 1);
	Matrix initVgabc = Matrix::Zero(3, 1);


	Matrix Tabc_dq = getParkTransformMatrix(mThetaPLL);
	Matrix Td = Tabc_dq.row(0);
	Matrix Tq = Tabc_dq.row(1);
	Matrix Tdq_abc = getInverseParkTransformMatrix(mThetaPLL);
	Matrix Tabc1 = Tdq_abc.col(0);
	Matrix Tabc2 = Tdq_abc.col(1);
	mA <<
		0, mKiPLL, 0, 0, 0, 0, 0, 0, Tq, Matrix::Zero(1, 3),
		0, 0, 0, 0, 0, 0, 0, 0, Tq, Matrix::Zero(1, 3),
		0, 0, -mOmegaCutoff, 0, 0, 0, 0, 0, Matrix::Zero(1, 3), Matrix::Zero(1, 3),
		0, 0, 0, -mOmegaCutoff, 0, 0, 0, 0, Matrix::Zero(1, 3), Matrix::Zero(1, 3),
		0, 0, -1, 0, 0, 0, 0, 0, Matrix::Zero(1, 3), Matrix::Zero(1, 3),
		0, 0, 0, -1, 0, 0, 0, 0, Matrix::Zero(1, 3), Matrix::Zero(1, 3),
		0, 0, -mKpPowerCtrld, 0, mKiPowerCtrld, 0, 0, 0, Matrix::Zero(1, 3), -Td,//phi_d
		0, 0, 0, -mKpPowerCtrlq, 0, mKiPowerCtrlq, 0, 0, Matrix::Zero(1, 3), -Tq,//phi_q
		// Vc_abc
		Matrix::Zero(3,8), 1. / mCf / mRc * Matrix::Identity(3, 3), 1. / mCf * Matrix::Identity(3,3),
		// If_abc
		Matrix::Zero(3, 1), Matrix::Zero(3, 1), -Tabc1 * mKpCurrCtrld * mKpPowerCtrld, -Tabc2*mKpCurrCtrlq*mKpPowerCtrlq,
			Tabc1* mKpCurrCtrld* mKiPowerCtrld, Tabc2*mKpCurrCtrlq*mKiPowerCtrlq,
			Tabc1* mKiCurrCtrld, Tabc2*mKiCurrCtrlq, Matrix::Zero(3, 3),
			-Tabc1 * mKpCurrCtrld*Td-Tabc2*mKpCurrCtrlq*Tq;

		mB <<
			1, 0, 0, Matrix::Zero(1, 3),
			0, 0, 0, Matrix::Zero(1, 3),
			0, 0, 0, 3. / 2. * mOmegaCutoff * sqrt(3)*(Td * mIntfCurrent * Td + Tq * mIntfCurrent * Tq),
			0, 0, 0, 3. / 2. * mOmegaCutoff * sqrt(3)*(Tq * mIntfCurrent * Td - Td * mIntfCurrent * Tq),
			0, 1, 0, Matrix::Zero(1, 3),
			0, 0, 1, Matrix::Zero(1, 3),
			0, 0, 0, Matrix::Zero(1, 3),
			0, 0, 0, Matrix::Zero(1, 3),
			Matrix::Zero(3, 1), Matrix::Zero(3, 1), Matrix::Zero(3, 1), -1 / mCf / mRc * Matrix::Identity(3, 3),
			Matrix::Zero(3, 1), Tabc1 * mKpCurrCtrld * mKpPowerCtrld, Tabc2* mKpCurrCtrlq* mKpPowerCtrlq, Matrix::Zero(3, 3);

		mC <<
			Matrix::Zero(3, 8), 1 / mRc * Matrix::Identity(3, 3), Matrix::Zero(3, 3);

		mD <<
			Matrix::Zero(3, 1), Matrix::Zero(3, 1), Matrix::Zero(3, 1), -1 / mRc * Matrix::Identity(3, 3);

		// #### initialize states ####
		// initialize interface voltage at inverter terminal
		initVgabc = mVcabc;

		mThetaPLL = 0;
		mPhiPLL = -0.000060;
		mP = sqrt(3)*(3. / 2. * (Td * mIntfCurrent * Td *mVcabc + Tq * mIntfCurrent * Tq * mVcabc)).coeff(0, 0);
		mQ = sqrt(3)*(3. / 2. * (Tq * mIntfCurrent * Td *mVcabc - Td * mIntfCurrent * Tq *mVcabc)).coeff(0, 0);
		mPhi_d = 113.612992;
		mPhi_q = 22.045339;
		mGamma_d = 0.155978;
		mGamma_q = 0.006339;
		mVcabc = initVgabc;
		mIfabc = Matrix::Zero(3, 1);
		/*mVca = initVcabc(0, 0);
		mVcb = initVcabc(1, 0);
		mVcc = initVcabc(2, 0);
		mIfa = initIfabc(0, 0);
		mIfb = initIfabc(1, 0);
		mIfc = initIfabc(2, 0);*/

		mStates <<
			mThetaPLL, mPhiPLL, mP, mQ, mPhi_d, mPhi_q, mGamma_d,
			mGamma_q, mVcabc, mIfabc;

		mU <<
			omega, mPref, mQref, initVgabc;
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::updateStates() {

	Matrix newStates = Matrix::Zero(14, 1);
	Matrix newU = Matrix::Zero(6, 1);

	newU <<
		mOmegaN, mPref, mQref, mIntfVoltage;

	newStates = Math::StateSpaceTrapezoidal(mStates, mA, mB, mTimeStep, newU, mU);

	// update states
	mThetaPLL = newStates(0, 0);
	mPhiPLL = newStates(1, 0);
	mP = newStates(2, 0);
	mQ = newStates(3, 0);
	mPhi_d = newStates(4, 0);
	mPhi_q = newStates(5, 0);
	mGamma_d = newStates(6, 0);
	mGamma_d = newStates(7, 0);
	mVcabc = newStates.block(8, 0, 3, 1);
	mIfabc = newStates.block(11, 0, 3, 1);
/*
	mIfa = newStates(8, 1);
	mIfb = newStates(9, 1);
	mIfc = newStates(10, 1);
	mVca = newStates(11, 1);
	mVcb = newStates(12, 1);
	mVcc = newStates(13, 1);
*/
	mStates = newStates;

	mU = newU;

	// update coefficients in A, B matrices due to linearization
	updateLinearizedCoeffs();
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::updateLinearizedCoeffs() {

	Matrix Tabc_dq = getParkTransformMatrix(mThetaPLL);
	Matrix Td = Tabc_dq.row(0);
	Matrix Tq = Tabc_dq.row(1);
	Matrix Tdq_abc = getInverseParkTransformMatrix(mThetaPLL);
	Matrix Tabc1 = Tdq_abc.col(0);
	Matrix Tabc2 = Tdq_abc.col(1);

	mA.block(0, 8, 1, 3) = Tq;
	mA.block(1, 8, 1, 3) = Tq;
	mA.block(6, 11, 1, 3) = -Td;
	mA.block(7, 11, 1, 3) = -Tq;

	Matrix A_bottom = Matrix::Zero(3, 14);
	A_bottom<<
		Matrix::Zero(3, 1), Matrix::Zero(3, 1), -Tabc1 * mKpCurrCtrld * mKpPowerCtrld, -Tabc2 * mKpCurrCtrlq * mKpPowerCtrlq,
		Tabc1* mKpCurrCtrld* mKiPowerCtrld, Tabc2* mKpCurrCtrlq* mKiPowerCtrlq,
		Tabc1* mKiCurrCtrld, Tabc2* mKiCurrCtrlq, Matrix::Zero(3, 3),
		-Tabc1 * mKpCurrCtrld * Td - Tabc2 * mKpCurrCtrlq * Tq;
	mA.block(11, 0, 3, 14) = A_bottom;
	/*
	 will it be faster to reconstruct the full B matrix (14x5)
	 rather than doing three insertions?
	*/
	mB.block(2, 2, 1, 3) = 3. / 2. * mOmegaCutoff * (Td * mIg_abc * Td + Tq * mIg_abc * Tq);
	mB.block(3, 2, 1, 3) = 3. / 2. * mOmegaCutoff * (Tq * mIg_abc * Td - Td * mIg_abc * Tq);
	Matrix B_bottom = Matrix::Zero(3, 6);
	B_bottom<<
		Matrix::Zero(3, 1), Tabc1 * mKpCurrCtrld * mKpPowerCtrld, Tabc2* mKpCurrCtrlq* mKpPowerCtrlq, Matrix::Zero(3, 3);
	mB.block(11, 0, 3, 6) = B_bottom;
}

Matrix EMT::Ph3::AvVoltSourceInverterStateSpace::parkTransform(Real theta, Real fa, Real fb, Real fc) {

	Matrix dqvector(2, 1);
	// Park transform
	Real d, q;

	d = 2. / 3. * sin(theta) * fa + 2. / 3. * sin(theta - 2. * M_PI / 3.) * fb + 2. / 3. * sin(theta + 2. * M_PI / 3.) * fc;
	q = 2. / 3. * cos(theta) * fa + 2. / 3. * cos(theta - 2. * M_PI / 3.) * fb + 2. / 3. * cos(theta + 2. * M_PI / 3.) * fc;

	dqvector << d,
		q;

	return dqvector;

}

Matrix EMT::Ph3::AvVoltSourceInverterStateSpace::getParkTransformMatrix(Real theta) {
	Matrix Tdq = Matrix::Zero(2, 3);
	Tdq <<
		2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
		2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.);

	return Tdq;
}
Matrix EMT::Ph3::AvVoltSourceInverterStateSpace::inverseParkTransform(Real theta, Real fd, Real fq, Real zero) {

	Matrix abcVector(3, 1);

	// inverse Park transform
	Real a, b, c;

	a = sin(theta) * fd + cos(theta) * fq + 1. * zero;
	b = sin(theta - 2. * M_PI / 3.) * fd + cos(theta - 2. * M_PI / 3.) * fq + 1. * zero;
	c = sin(theta + 2. * M_PI / 3.) * fd + cos(theta + 2. * M_PI / 3.) * fq + 1. * zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}

Matrix EMT::Ph3::AvVoltSourceInverterStateSpace::getInverseParkTransformMatrix(Real theta) {
	Matrix Tabc = Matrix::Zero(3, 2);
	Tabc <<
		sin(theta), cos(theta),
		sin(theta - 2. * M_PI / 3.), cos(theta - 2. * M_PI / 3.),
		sin(theta + 2. * M_PI / 3.), cos(theta + 2. * M_PI / 3.);

	return Tabc;
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::mnaInitialize(Real omega, Real timeStep,
	Attribute<Matrix>::Ptr leftVector){
	updateSimNodes();
	Complex voltageRef = attribute<Complex>("V_ref")->get();
	mIntfVoltage(0, 0) = voltageRef.real() * cos(Math::phase(voltageRef));
	mIntfVoltage(1, 0) = voltageRef.real() * cos(Math::phase(voltageRef) - 2. / 3. * M_PI);
	mIntfVoltage(2, 0) = voltageRef.real() * cos(Math::phase(voltageRef) + 2. / 3. * M_PI);
	mIntfCurrent = Matrix::Zero(3, 1);
	initializeStates(omega, timeStep, leftVector);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}
void EMT::Ph3::AvVoltSourceInverterStateSpace::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Apply matrix stamp for equivalent resistance
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), mYc);
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), mYc);
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), mYc);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), mYc);
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), mYc);
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), mYc);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -mYc);
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -mYc);

		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -mYc);
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -mYc);

		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -mYc);
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -mYc);
	}
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Apply matrix stamp for equivalent current source
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, simNode(0, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(0, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(0, 2), -mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, simNode(1, 0), mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(1, 1), mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(1, 2), mEquivCurrent(2, 0));
	}
}


void EMT::Ph3::AvVoltSourceInverterStateSpace::MnaPreStep::execute(Real time, Int timeStepCount) {
	mAvVoltSourceInverterStateSpace.updateEquivCurrent(time);
	mAvVoltSourceInverterStateSpace.mnaApplyRightSideVectorStamp(mAvVoltSourceInverterStateSpace.mRightVector);
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::MnaPostStep::execute(Real time, Int timeStepCount) {
	mAvVoltSourceInverterStateSpace.mnaUpdateVoltage(*mLeftVector);
	mAvVoltSourceInverterStateSpace.updateStates();
	mAvVoltSourceInverterStateSpace.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::mnaUpdateVoltage(const Matrix& leftVector) {
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, simNode(1, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, simNode(1, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, simNode(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::realFromVectorElement(leftVector, simNode(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::realFromVectorElement(leftVector, simNode(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::realFromVectorElement(leftVector, simNode(0, 2));
	}
}


void EMT::Ph3::AvVoltSourceInverterStateSpace::mnaUpdateCurrent(const Matrix& leftVector) {
	// signs are not verified
	mIntfCurrent(0, 0) = mEquivCurrent(0, 0) - mIntfVoltage(0, 0) / mRc;
	mIntfCurrent(1, 0) = mEquivCurrent(1, 0) - mIntfVoltage(1, 0) / mRc;
	mIntfCurrent(2, 0) = mEquivCurrent(2, 0) - mIntfVoltage(2, 0) / mRc;
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::updateEquivCurrent(Real time) {
	mEquivCurrent = mVcabc / mRc;
}
