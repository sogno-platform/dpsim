/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>
#include <dpsim-models/Solver/MNANonlinearVariableCompInterface.h>

namespace CPS{
namespace EMT{
namespace Ph3{
class SynchronGenerator4OrderSSN:  
    public Base::ReducedOrderSynchronGenerator<Real>,
    public MNANonlinearVariableCompInterface,
    public SharedFactory<SynchronGenerator4OrderSSN> {

public:

	SynchronGenerator4OrderSSN(String uid, String name, Logger::Level logLevel = Logger::Level::off);

	SynchronGenerator4OrderSSN(String name, Logger::Level logLevel = Logger::Level::off);

    virtual ~SynchronGenerator4OrderSSN() {};

    virtual void calculateNonlinearFunctionResult() override;
    virtual void mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) override;
    virtual void mnaCompPostStep(const Matrix &leftVector) override;
    virtual void mnaCompUpdateVoltage(const Matrix& leftVector) override;
    virtual void mnaCompUpdateCurrent(const Matrix& leftVector) override;
    virtual void mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
    virtual void mnaCompPreStep(Real time, Int timeStepCount) override;
    virtual void mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
    virtual void mnaCompApplyRightSideVectorStamp(Matrix& rightVector) override;
    virtual void mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
    void updateJacobian();
    void updateCurrentStates();
    void updateOldStates();
    void updateImplicitStates(const Matrix& leftVector);
    virtual void iterationUpdate(const Matrix& leftVector) override;

    virtual void initializeResistanceMatrix() override {};

    virtual void specificInitialization() override;

    virtual void stepInPerUnit() override {};

    virtual bool hasParameterChanged() override {return true;};

protected:
    Matrix Jacobian = Matrix::Zero(4,4);

    //inputs
    double P_mech;
    double P_mech_old;
    double Ef;
    double Ef_old;

    //states
    double theta;
    double theta_old;
    double Ed;
    double Ed_old;
    double Eq;
    double Eq_old;
    double omega;
    double omega_old;

    //helpers
    double Vd = 0;
    double Vq = 0;
    double Vd_old = 0;
    double Vq_old = 0;

private:
    //constants
    double C_d;
    double C_dd;
    double C_0dd;
    double C_qq;
    double C_0qq;
    double C_wbq;
    double C_wbd;
    double C_wb;
    double C_h;
};
}
}
}

