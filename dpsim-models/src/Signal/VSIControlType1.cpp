#include <dpsim-models/Signal/VSIControlType1.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

void VSIControlType1::setParameters(std::shared_ptr<Base::VSIControlParameters> parameters)
{
	if (auto params = std::dynamic_pointer_cast<Signal::VSIControlType1Parameters>(parameters))
	{
		mParameters = params;
		SPDLOG_LOGGER_INFO(mSLog,
						   "\nVSIController Type1 Parameters: "
						   "\nKpv: {:e}"
						   "\nKiv: {:e}"
						   "\nKpc: {:e}"
						   "\nKic: {:e}",
						   mParameters->Kpv, mParameters->Kiv,
						   mParameters->Kpc, mParameters->Kic);
		mSLog->flush();
	}
	else
	{
		std::cout << "Type of parameters class of " << this->name() << " has to be VSIControlType1!" << std::endl;
		throw CPS::TypeException();
	}
}

void VSIControlType1::initialize(const Complex &Vsref_dq, const Complex &Vcap_dq,
								 const Complex &Ifilter_dq, Real time_step, Bool modelAsCurrentSource)
{
	mTimeStep = time_step;
	mModelAsCurrentSource = modelAsCurrentSource;

	mVcap_dq = Vcap_dq;
	mIfilter_dq = Ifilter_dq;

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

	mA << 0, 0, 0, 0,
		0, 0, 0, 0,
		mParameters->Kiv, 0, 0, 0,
		0, mParameters->Kiv, 0, 0;

	mB << 1, 0, -1, 0, 0, 0,
		0, 1, 0, -1, 0, 0,
		mParameters->Kpv, 0, -mParameters->Kpv, 0, -1, 0,
		0, mParameters->Kpv, 0, -mParameters->Kpv, 0, -1;

	mC << mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic, 0,
		0, mParameters->Kpc * mParameters->Kiv, 0, mParameters->Kic;

	mD << mParameters->Kpc * mParameters->Kpv, 0, -mParameters->Kpc * mParameters->Kpv + 1, 0, -mParameters->Kpc, 0,
		0, mParameters->Kpc * mParameters->Kpv, 0, -mParameters->Kpc * mParameters->Kpv + 1, 0, -mParameters->Kpc;

	Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, Matrix::Zero(4, 1), mTimeStep, mATrapezoidal, mBTrapezoidal, mCTrapezoidal);

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
}

void VSIControlType1::calculateVBRconstants()
{
	mA_VBR = mParameters->Kpc + mParameters->Kic * mTimeStep / 2.;
	mB_VBR = mParameters->Kpv + mParameters->Kiv * mTimeStep / 2.;
	mB2_VBR = mParameters->Kpv + mParameters->Kiv * mTimeStep;
	mC_VBR = mParameters->Kiv * mTimeStep / 2.;
	mD_VBR = mParameters->Kic * mTimeStep / 2.;
	mE_VBR = -mA_VBR * mB_VBR + 1.;

	// initialize Iref
	mIref_d = mB2_VBR * mParameters->VdRef - mB_VBR * mVcap_dq.real() - mC_VBR * mVcap_dq.real() + **mPhi_d * mParameters->Kiv;
	mIref_q = mB2_VBR * mParameters->VqRef - mB_VBR * mVcap_dq.imag() - mC_VBR * mVcap_dq.imag() + **mPhi_q * mParameters->Kiv;

	// calculate Vhist at t=k+1
	Real Vhist_d = -mA_VBR * mC_VBR * mVcap_dq.real() - mD_VBR * mIfilter_dq.real() + mD_VBR * mIref_d + mA_VBR * **mPhi_d * mParameters->Kiv + **mGamma_d * mParameters->Kic + mA_VBR * mB2_VBR * mParameters->VdRef;
	Real Vhist_q = -mA_VBR * mC_VBR * mVcap_dq.imag() - mD_VBR * mIfilter_dq.imag() + mD_VBR * mIref_q + mA_VBR * **mPhi_q * mParameters->Kiv + **mGamma_q * mParameters->Kic + mA_VBR * mB2_VBR * mParameters->VqRef;

	// Test
	Real Vsource_test_d = mVcap_dq.real() - mA_VBR * mB_VBR * mVcap_dq.real() - mA_VBR * mIfilter_dq.real() + Vhist_d;
	Real Vsource_test_q = mVcap_dq.imag() - mA_VBR * mB_VBR * mVcap_dq.imag() - mA_VBR * mIfilter_dq.imag() + Vhist_q;

	// intial reference current
	mIfilter_dq = Complex(mIref_d, mIref_q);

	SPDLOG_LOGGER_INFO(mSLog,
					   "\n--- VBR constants: ---"
					   "\nA_VBR = {}"
					   "\nB_VBR = {}"
					   "\nC_VBR = {}"
					   "\nD_VBR = {}"
					   "\nE_VBR = {}"
					   "\nIref_d = {}"
					   "\nIref_d = {}"
					   "\nInit Vhist_d = {}"
					   "\nInit Vhist_q = {}"
					   "\nInit Vsource_test_d = {}"
					   "\nInit Vsource_test_q = {}",
					   mA_VBR, mB_VBR, mC_VBR, mD_VBR, mE_VBR,
					   mIref_d, mIref_q, Vhist_d, Vhist_q,
					   Vsource_test_d, Vsource_test_q);
	mSLog->flush();
}

Complex VSIControlType1::step(const Complex &Vcap_dq, const Complex &Ifilter_dq)
{
	//
	**mStatePrev = **mStateCurr;

	// get current inputs
	**mInputPrev = **mInputCurr;
	**mInputCurr << mParameters->VdRef, mParameters->VqRef, Vcap_dq.real(), Vcap_dq.imag(), Ifilter_dq.real(), Ifilter_dq.imag();

	// calculate new states
	//**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
	**mStateCurr = Math::applyStateSpaceTrapezoidalMatrices(mATrapezoidal, mBTrapezoidal, mCTrapezoidal, **mStatePrev, **mInputCurr, **mInputPrev);

	// calculate controller outputs
	if (mModelAsCurrentSource)
	{
		Real error_d = mParameters->VdRef - Vcap_dq.real();
		Real error_q = mParameters->VqRef - Vcap_dq.imag();
		(**mOutput)(0, 0) = ((**mStateCurr)(0, 0) * mParameters->Kiv + mParameters->Kpv * error_d) * (mTimeStep / mParameters->tau) + (1. - mTimeStep / mParameters->tau) * Ifilter_dq.real();
		(**mOutput)(1, 0) = ((**mStateCurr)(1, 0) * mParameters->Kiv + mParameters->Kpv * error_q) * (mTimeStep / mParameters->tau) + (1. - mTimeStep / mParameters->tau) * Ifilter_dq.imag();
	}
	else
	{
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
	return Complex((**mOutput)(0, 0), (**mOutput)(1, 0));
}

Complex VSIControlType1::stepVBR(const Complex &Vcap_dq, const Complex &Ifilter_dq)
{
	// store previous values (t=k-1)
	Complex Vcap_dq_prev = mVcap_dq;
	Complex Ifilter_dq_prev = mIfilter_dq;
	Real Iref_d_prev = mIref_d;
	Real Iref_q_prev = mIref_q;

	// update variables at t=k
	mVcap_dq = Vcap_dq;
	mIfilter_dq = Ifilter_dq;

	// calculate reference current at time t=k
	mIref_d = mB2_VBR * mParameters->VdRef - mB_VBR * mVcap_dq.real() - mC_VBR * Vcap_dq_prev.real() + **mPhi_d * mParameters->Kiv;
	mIref_q = mB2_VBR * mParameters->VqRef - mB_VBR * mVcap_dq.imag() - mC_VBR * Vcap_dq_prev.imag() + **mPhi_q * mParameters->Kiv;

	// Update phi at time t=k
	**mPhi_d = **mPhi_d + (mTimeStep / 2.) * (2 * mParameters->VdRef - mVcap_dq.real() - Vcap_dq_prev.real());
	**mPhi_q = **mPhi_q + (mTimeStep / 2.) * (2 * mParameters->VqRef - mVcap_dq.imag() - Vcap_dq_prev.imag());

	// Update lambda at time t=k
	**mGamma_d = **mGamma_d + (mTimeStep / 2.) * (mIref_d - mIfilter_dq.real() + Iref_d_prev - Ifilter_dq_prev.real());
	**mGamma_q = **mGamma_q + (mTimeStep / 2.) * (mIref_q - mIfilter_dq.imag() + Iref_q_prev - Ifilter_dq_prev.imag());

	// calculate Vvbr at t=k+1
	Real Vhist_d = -mA_VBR * mC_VBR * mVcap_dq.real() - mD_VBR * mIfilter_dq.real() + mD_VBR * mIref_d + mA_VBR * **mPhi_d * mParameters->Kiv + **mGamma_d * mParameters->Kic + mA_VBR * mB2_VBR * mParameters->VdRef;
	Real Vhist_q = -mA_VBR * mC_VBR * mVcap_dq.imag() - mD_VBR * mIfilter_dq.imag() + mD_VBR * mIref_q + mA_VBR * **mPhi_q * mParameters->Kiv + **mGamma_q * mParameters->Kic + mA_VBR * mB2_VBR * mParameters->VqRef;

	// Test
	Real Vsource_test_d = mVcap_dq.real() - mA_VBR * mB_VBR * mVcap_dq.real() - mA_VBR * mIfilter_dq.real() + Vhist_d;
	Real Vsource_test_q = mVcap_dq.imag() - mA_VBR * mB_VBR * mVcap_dq.imag() - mA_VBR * mIfilter_dq.imag() + Vhist_q;
	//

	SPDLOG_LOGGER_DEBUG(mSLog,
						"\n--- Test Step: ---"
						"\nmVcap_dq_d = {}"
						"\nmVcap_dq_q = {}"
						"\nIfilter_dq_d = {}"
						"\nIfilter_dq_q = {}"
						"\nVsource_test_d = {}"
						"\nVsource_test_q = {}"
						"\nVhist_d = {}"
						"\nVhist_q = {}"
						"\nIref_d = {}"
						"\nIref_d = {}"
						"\nmPhi_d = {}"
						"\nmPhi_q = {}"
						"\nmGamma_d = {}"
						"\nmGamma_q = {}",
						mVcap_dq.real(), mVcap_dq.imag(),
						Ifilter_dq.real(), Ifilter_dq.imag(),
						Vsource_test_d, Vsource_test_q,
						Vhist_d, Vhist_q, mIref_d, mIref_q,
						**mPhi_d, **mPhi_q, **mGamma_d, **mGamma_q);

	return Complex(Vhist_d, Vhist_q);
}