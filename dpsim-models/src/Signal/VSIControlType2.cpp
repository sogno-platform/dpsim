#include <dpsim-models/Signal/VSIControlType2.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

void VSIControlType2::setParameters(std::shared_ptr<Base::VSIControlParameters> parameters) {
    if (auto params = std::dynamic_pointer_cast<Signal::VSIControlType2Parameters>(parameters)){
        mParameters = params;
        SPDLOG_LOGGER_INFO(mSLog, 
			"\nVSIController Type2 Parameters: "
			"\nKpv: {:e}"
			"\nKiv: {:e}"
			"\nKpc: {:e}"
			"\nKic: {:e}",
			mParameters->Kpv, mParameters->Kiv,
			mParameters->Kpc, mParameters->Kic);
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be VSIControlType2!" << std::endl;
		throw CPS::TypeException();
	}
}

void VSIControlType2::initialize(const Complex& Vsref_dq, const Complex& Vcap_dq, 
	const Complex& Ifilter_dq, Real time_step, Bool modelAsCurrentSource) {

	mTimeStep = time_step;
	mModelAsCurrentSource = modelAsCurrentSource;
	**mPhi_d = (Ifilter_dq.real() + mParameters->omegaNom * mParameters->Cf * Vcap_dq.imag()) / mParameters->Kiv;
	**mPhi_q = (Ifilter_dq.imag() - mParameters->omegaNom * mParameters->Cf * Vcap_dq.real()) / mParameters->Kiv; 
	**mGamma_d = (Vsref_dq.real() + mParameters->omegaNom * mParameters->Lf * Ifilter_dq.imag()) / mParameters->Kic;
	**mGamma_q = (Vsref_dq.imag() - mParameters->omegaNom * mParameters->Lf * Ifilter_dq.real()) / mParameters->Kic;

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
		mParameters->Kpv, 0, -mParameters->Kpv, -mParameters->omegaNom * mParameters->Cf, -1, 0,
		0, mParameters->Kpv, mParameters->omegaNom * mParameters->Cf, -mParameters->Kpv, 0, -1;

	mC <<
		mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic, 0,
		0, mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic;

	mD <<
		mParameters->Kpc * mParameters->Kpv , 0, -mParameters->Kpc * mParameters->Kpv, -mParameters->Kpc * mParameters->omegaNom * mParameters->Cf, -mParameters->Kpc, -mParameters->omegaNom * mParameters->Lf,
		0, mParameters->Kpc * mParameters->Kpv, mParameters->Kpc * mParameters->omegaNom * mParameters->Cf, -mParameters->Kic * mParameters->Kpv, mParameters->omegaNom * mParameters->Lf, -mParameters->Kpc; 
	
	Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, Matrix::Zero(4,1), mTimeStep, mATrapezoidal, mBTrapezoidal, mCTrapezoidal);

	**mInputCurr << mParameters->VdRef, mParameters->VqRef, Vcap_dq.real(), Vcap_dq.imag(), Ifilter_dq.real(), Ifilter_dq.imag();

    // Log state-space matrices
	SPDLOG_LOGGER_INFO(mSLog, 
				"\nState space matrices:"
				"\nA = \n{}"
    			"\nB = \n{}"
    			"\nC = \n{}"
    			"\nD = \n{}", 
				mA, mB, mC, mD);
    mSLog->flush();

	if (mModelAsCurrentSource)
		mAuxVar = mTimeStep / mParameters->tau;
}

Complex VSIControlType2::step(const Complex& Vcap_dq, const Complex& Ifilter_dq) {
    
	//
	**mStatePrev = **mStateCurr;

    // get current inputs
	**mInputPrev = **mInputCurr;
    **mInputCurr << mParameters->VdRef, mParameters->VqRef, Vcap_dq.real(), Vcap_dq.imag(), Ifilter_dq.real(), Ifilter_dq.imag();

	// calculate new states
	//**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    **mStateCurr = Math::applyStateSpaceTrapezoidalMatrices(mATrapezoidal, mBTrapezoidal, mCTrapezoidal, **mStatePrev, **mInputCurr, **mInputPrev);

	if (mModelAsCurrentSource) {
		Real error_d = mParameters->VdRef - Vcap_dq.real();
		Real error_q = mParameters->VqRef - Vcap_dq.imag();
		(**mOutput)(0,0) = ((**mStateCurr)(0, 0) * mParameters->Kiv + mParameters->Kpv * error_d - mParameters->omegaNom * mParameters->Cf * Vcap_dq.imag()) * mAuxVar + (1 - mAuxVar) * Ifilter_dq.real();
		(**mOutput)(1,0) = ((**mStateCurr)(1, 0) * mParameters->Kiv + mParameters->Kpv * error_q + mParameters->omegaNom * mParameters->Cf * Vcap_dq.real()) * mAuxVar + (1 - mAuxVar) * Ifilter_dq.imag();
	}
	else {
		// calculate new outputs
		**mOutput = mC * **mStateCurr + mD * **mInputCurr;
	}
    
	SPDLOG_LOGGER_DEBUG(mSLog, 
				"\n - InputCurr = \n{}"
				"\n - InputPrev = \n{}"
				"\n - StatePrev = \n{}"
				"\n - StateCurr = \n{}"
				"\n - Output values: \n{}",
				**mInputCurr, **mInputPrev, **mStatePrev, **mStateCurr, **mOutput);
				
    //
	return Complex((**mOutput)(0,0), (**mOutput)(1,0));
}

