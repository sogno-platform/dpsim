#include <dpsim-models/Signal/VSIControlType3.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

void VSIControlType3::setParameters(std::shared_ptr<Base::VSIControlParameters> parameters) {
    if (auto params = std::dynamic_pointer_cast<Signal::VSIControlType3Parameters>(parameters)){
        mParameters = params;
        SPDLOG_LOGGER_INFO(mSLog, 
			"\nVSIController Type3 Parameters: "
			"\nKpv: {:e}"
			"\nKiv: {:e}"
			"\nKpc: {:e}"
			"\nKic: {:e}",
			mParameters->Kpv, mParameters->Kiv,
			mParameters->Kpc, mParameters->Kic);
		mSLog->flush();
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be VSIControlType3!" << std::endl;
		throw CPS::TypeException();
	}
}

void VSIControlType3::initialize(const Complex& Vsref_dq, const Complex& Vcap_dq, 
	const Complex& Ifilter_dq, Real time_step) {

	mTimeStep = time_step;
	**mPhi_d = Ifilter_dq.real() / mParameters->Kiv;
	**mPhi_q = Ifilter_dq.imag() / mParameters->Kiv; 
	**mGamma_d = (Vsref_dq.real() - Vcap_dq.real()) / mParameters->Kic;
	**mGamma_q = (Vsref_dq.imag() - Vcap_dq.imag()) / mParameters->Kic;

	SPDLOG_LOGGER_INFO(mSLog,
			"\nInitialize controller states:"	  
			"\n\tPhi_d = {}"
			"\n\tPhi_q = {}"
			"\n\tGamma_d = {}"
			"\n\tGamma_q = {}",
			**mPhi_d, **mPhi_q, **mGamma_d, **mGamma_q);
	mSLog->flush();

	**mStateCurr << **mPhi_d, **mPhi_q, **mGamma_d, **mGamma_q;

    // initialize state matrix
    // [x] = [phid, phiq, gammad, gammaq]
	// [u] = [vdref, vqref, vdc, vqc, idc, idq]
	// [y] = [vdout, vqout]

    mA <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		mParameters->Kiv, 0, 0, 0,
		0, mParameters->Kiv, 0, 0;

	mB <<
		1, 0, -1, 0, 0, 0,
		0, 1, 0, -1, 0, 0,
		mParameters->Kpv, 0, -mParameters->Kpv, 0, -1, 0,
		0, mParameters->Kpv, 0, -mParameters->Kpv, 0, -1;

	mC <<
		mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic, 0,
		0, mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic;

	mD <<
		mParameters->Kpc * mParameters->Kpv , 0, -mParameters->Kpc * mParameters->Kpv + 1, 0, -mParameters->Kpc, 0,
		0, mParameters->Kpc * mParameters->Kpv, 0, -mParameters->Kpc * mParameters->Kpv + 1, 0, -mParameters->Kpc;

    Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, Matrix::Zero(4,1), mTimeStep, mATrapezoidal, mBTrapezoidal, mCTrapezoidal);

	**mInputCurr << mParameters->VdRef, mParameters->VqRef, Vcap_dq.real(), Vcap_dq.imag(), Ifilter_dq.real(), Ifilter_dq.imag();

    // Log state-space matrices
	SPDLOG_LOGGER_INFO(mSLog, "State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog, "A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog, "B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog, "C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog, "D = \n{}", mD);
    mSLog->flush();
}

Complex VSIControlType3::step(const Complex& Vcap_dq, const Complex& Ifilter_dq) {
    
	//
	**mStatePrev = **mStateCurr;

    // get current inputs
    **mInputCurr << mParameters->VdRef, mParameters->VqRef, Vcap_dq.real(), Vcap_dq.imag(), Ifilter_dq.real(), Ifilter_dq.imag();
	SPDLOG_LOGGER_DEBUG(mSLog, "Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, **mInputCurr, **mInputPrev, **mStatePrev);

	// calculate new states
	//**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    **mStateCurr = Math::applyStateSpaceTrapezoidalMatrices(mATrapezoidal, mBTrapezoidal, mCTrapezoidal, **mStatePrev, **mInputCurr, **mInputPrev);
	SPDLOG_LOGGER_DEBUG(mSLog, "stateCurr = \n {}", **mStateCurr);

	// calculate new outputs
	//**mOutput = mC * **mStateCurr + mD * **mInputCurr;
	//SPDLOG_LOGGER_DEBUG(mSLog, "Output values: outputCurr = \n{}", **mOutput);
    
	Real theta_s = 1e-3;
	Real error_d = mParameters->VdRef - Vcap_dq.real();
	Real error_q = mParameters->VqRef - Vcap_dq.imag();
	(**mOutput)(0,0) = ((**mStateCurr)(0, 0) * mParameters->Kiv + mParameters->Kpv * error_d) * (mTimeStep / theta_s) + (1. - mTimeStep / theta_s) * Ifilter_dq.real();
	(**mOutput)(1,0) = ((**mStateCurr)(1, 0) * mParameters->Kiv + mParameters->Kpv * error_q) * (mTimeStep / theta_s) + (1. - mTimeStep / theta_s) * Ifilter_dq.imag();


    //
	return Complex((**mOutput)(0,0), (**mOutput)(1,0));
}

