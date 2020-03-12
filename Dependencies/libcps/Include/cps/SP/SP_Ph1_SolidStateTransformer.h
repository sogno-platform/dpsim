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

#pragma once

#include <cps/PowerComponent.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/SP/SP_Ph1_Load.h>


namespace CPS {
namespace SP { namespace Ph1 {
    /* \brief Ideal solid state transformer
    * Modelled as two loads at each side. 
    * Demands Pref, Q1 from side 1, and generate Pref, Q2 to side 2. 
    * Depends on the actual condition, values can be negative
    */ 
	class SolidStateTransformer :
		public PowerComponent<Complex>,
		public SharedFactory<SolidStateTransformer>,
		public MNAInterface{
	private:
    ///
    std::shared_ptr<SP::Ph1::Load> mSubLoadSide1;
    ///
    std::shared_ptr<SP::Ph1::Load> mSubLoadSide2;

    /// Rated Apparent Power [VA]
    Real mRatedPower = 0;
    /// Power [watt]
    Real mPref = std::numeric_limits<double>::infinity();
    /// Active power at secondary side [watt]
    Real mP2 = std::numeric_limits<double>::infinity();
    /// Reactive power at primary side [var]
    Real mQ1ref;
    /// Reactive power at secondary side [var]
    Real mQ2ref;
    /// Nominal voltage of primary side [V]
    Real mNominalVoltageEnd1;
    /// Nominal voltage of secondary side [V]
    Real mNominalVoltageEnd2;
    // Per Unit values
    /// Active power at primary side [p.u.]
    Real mPref_perUnit;
    /// Active power at secondary side [p.u.]
    Real mP2_perUnit;
    /// Reactive power at primary side [p.u.]
    Real mQ1ref_perUnit;
    /// Reactive power at secondary side [p.u.]
    Real mQ2ref_perUnit;
    
    public:
    ///
    SolidStateTransformer(String uid, String name, Logger::Level logLevel = Logger::Level::off);
    ///
    SolidStateTransformer(String name, Logger::Level logLevel = Logger::Level::off)
    :SolidStateTransformer(name, name, logLevel) {};
    ///
    PowerComponent<Complex>::Ptr clone(String name);

    // #### Power Flow Section ####
    /// Initializes component
    void initializeFromPowerflow(Real frequency);
    ///
    void setParameters(Real nomV1, Real nomV2, Real Pref, Real Q1ref, Real Q2ref);
    ///
    void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
    ///
    Complex getNodalInjection(CPS::TopologicalNode::Ptr node);

    // // #### MNA Section ####
    // /// Initializes internal variables of the component
    // void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
    // /// Stamps system matrix
    // void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
    // /// Updates internal current variable of the component
    // void mnaUpdateCurrent(const Matrix& leftVector);
    // /// Updates internal voltage variable of the component
    // void mnaUpdateVoltage(const Matrix& leftVector);

        };


}
}
}