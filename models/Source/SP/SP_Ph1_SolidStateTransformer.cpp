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

#include <cps/SP/SP_Ph1_SolidStateTransformer.h>

using namespace CPS;

SP::Ph1::SolidStateTransformer::SolidStateTransformer(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	mSLog->info("Create {} of type {}", mName, this->type());
	mSLog->flush();
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
    setTerminalNumber(2);

	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q1_ref", &mQ1ref, Flags::read | Flags::write);
	addAttribute<Real>("Q2_ref", &mQ2ref, Flags::read | Flags::write);
};

SimPowerComp<Complex>::Ptr SP::Ph1::SolidStateTransformer::clone(String name) {
	// everything set by initializeFromPowerflow
	return SolidStateTransformer::make(name, mLogLevel);
}

void SP::Ph1::SolidStateTransformer::setParameters(Real nomV1, Real nomV2, Real Pref, Real Q1ref, Real Q2ref){
    mNominalVoltageEnd1 = nomV1;
    mNominalVoltageEnd2 = nomV2;
    mPref = Pref;
    mQ1ref = Q1ref;
    mQ2ref = Q2ref;
    mP2 = -1 * std::sqrt(Pref * Pref + Q1ref * Q1ref - Q2ref * Q2ref);
}

void SP::Ph1::SolidStateTransformer::initializeFromPowerflow(Real frequency){
    checkForUnconnectedTerminals();
    if(std::isinf(mP2)){
        std::stringstream ss;
        ss << "SST >>" << this->name() << ": infinite or nan values. Or initialized before setting parameters.";
        throw std::invalid_argument(ss.str());
    }
    if ((mPref * mP2) > 0){
        throw std::invalid_argument("power at primary and secondary sides should be opposite");
    }
    mSubLoadSide1 = Load::make(mName + "_subLoad1", mLogLevel);
    mSubLoadSide1->setParameters(mPref, mQ1ref, mNominalVoltageEnd1);
    mSubLoadSide2 = Load::make(mName + "_subLoad2", mLogLevel);
    mSubLoadSide2->setParameters(mP2, mQ2ref, mNominalVoltageEnd2);
    mSubLoadSide1->connect({mTerminals[0]->node()});
    mSubLoadSide2->connect({mTerminals[1]->node()});

    mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nTerminal 0 power flow: {:s} VA"
		"\nTerminal 1 power flow: {:s} VA"
		"\n--- Initialization from powerflow finished ---",
		Logger::complexToString(Complex(mPref,mQ1ref)),
		Logger::complexToString(Complex(mP2,mQ2ref)));

}

void SP::Ph1::SolidStateTransformer::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
    mPref_perUnit = mPref / baseApparentPower;
    mP2_perUnit = mP2 / baseApparentPower;
    mQ1ref_perUnit = mQ1ref / baseApparentPower;
    mQ2ref_perUnit = mQ2ref / baseApparentPower;
    mSubLoadSide1->calculatePerUnitParameters(baseApparentPower, baseOmega);
    mSubLoadSide2->calculatePerUnitParameters(baseApparentPower, baseOmega);
    mSLog->info(
        "\n#### Calculate Per Unit Parameters for {}"
        "\nTerminal 0 power flow: {:s} p.u."
        "\nTerminal 1 power flow: {:s} p.u."
        "\n#### Calculate Per Unit Parameters finished ---",
        mName,
        Logger::complexToString(Complex(mPref_perUnit, mQ1ref_perUnit)),
        Logger::complexToString(Complex(mP2_perUnit, mQ2ref_perUnit)));
}

Complex SP::Ph1::SolidStateTransformer::getNodalInjection(CPS::TopologicalNode::Ptr node) {
    if (node->name() == mTerminals[0]->node()->name())
    {
        mSLog->info(
            "\n#### get nodal injection for primary side"
            "\nreturned {:s} p.u.",
            Logger::complexToString(Complex(mPref_perUnit, mQ1ref_perUnit)));
        return Complex(mPref_perUnit, mQ1ref_perUnit);
    }
    else if (node->name() == mTerminals[1]->node()->name())
    {
        mSLog->info(
            "\n#### get nodal injection for secondary side"
            "\nreturned {:s} p.u.",
            Logger::complexToString(Complex(mP2_perUnit, mQ2ref_perUnit)));
        return Complex(mP2_perUnit, mQ2ref_perUnit);
    }
    else{
        throw std::invalid_argument("Failed to process nodal power injection of Solid State Transformer"
        + this->name());
    }
}
