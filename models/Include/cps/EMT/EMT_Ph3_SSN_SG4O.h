
#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Solver/MNAVariableCompInterface.h>
#include <cps/Solver/SSNNonlinearCompInterface.h>
#include <cps/Base/Base_ReducedOrderSynchronGenerator.h>


namespace CPS{
    namespace EMT{
        namespace Ph3{
            namespace SSN{
                ///FIXME: Should only be usable by SSNSolver.
                class SG4O:
					public Base::ReducedOrderSynchronGenerator<Real>,
				    public MNAVariableCompInterface,
				    public SharedFactory<SG4O>,
                    public SSNNonlinearCompInterface
                    {
                public:
                    /// Defines UID, name, component parameters and logging level
				    SG4O(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				    /// Defines name and logging level
				    SG4O(String name, Logger::Level logLevel = Logger::Level::off)
											    : SG4O(name, name, logLevel) {}

				    SimPowerComp<Real>::Ptr clone(String name);

			    	// #### MNA section ####

				    /// Stamps system matrix
				    virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
				    /// Stamps right side (source) vector
				    virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
				    /// Update interface voltage from MNA system result
				    void mnaUpdateVoltage(const Matrix& leftVector);
				    /// Update interface current from MNA system result
				    void mnaUpdateCurrent(const Matrix& leftVector);
				    /// MNA post step operations
				    virtual void mnaPostStep(const Matrix& leftVector) override;
				    /// Add MNA pre step dependencies
				    void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
				    /// Add MNA post step dependencies
				    void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

					virtual void ssnUpdate(const Matrix& leftVector) override;

					void ssnUpdateInputs(Real time);

					void ssnUpdateJacobian(const Matrix& leftVector);

					void ssnUpdateNonExplicitStates(const Matrix& leftVector);
					
					void ssnCalculateFunctionResult(const Matrix& leftVector, Matrix& ssnFunctionResult);

					void ssnUpdateState();

					virtual Bool hasParameterChanged() {return true;}

					virtual void stepInPerUnit() override;
                    

				    class MnaPreStep : public Task {
				    public:
					    MnaPreStep(SG4O& SG4O) :
						    Task(**SG4O.mName + ".MnaPreStep"), mSG4O(SG4O) {
							    mSG4O.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
					    }
					    void execute(Real time, Int timeStepCount) { mSG4O.mnaPreStep(time, timeStepCount); };
				    private:
					    SG4O& mSG4O;
				    };

				    class MnaPostStep : public Task {
				    public:
					    MnaPostStep(SG4O& SG4O, Attribute<Matrix>::Ptr leftVector) :
					    	Task(**SG4O.mName + ".MnaPostStep"),
						    mSG4O(SG4O), mLeftVector(leftVector) {
						    	mSG4O.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
					    }
					    void execute(Real time, Int timeStepCount) { mSG4O.mnaPostStep(**mLeftVector); };
				    private:
				    	SG4O& mSG4O;
				    	Attribute<Matrix>::Ptr mLeftVector;
                    };
                protected:
					Real timeStep = 0.; // redundant

                    Real mX_d = 0.;
                    Real mX_dDash = 0.;
                    Real mX_q = 0.;
                    Real mX_qDash = 0.;
                    Real mT_d0Dash = 0.;
                    Real mT_q0Dash = 0.;
					//Real mH = 0.;
					Real mOmega_base = 0.;

					Real mE_f = 0.;
					Real me_f = 0.;
					Real mP_mech = 0.;
					Real mp_mech = 0.;

					//	Save iterative currents for mnaUpdate later
					Real mI_a = 0.;
					Real mI_b = 0.;
					Real mI_c = 0.;

                    Matrix State = Matrix::Zero(4, 1); //delta, E_d', E_q', omega
					Matrix nonExplicitState = Matrix::Zero(1, 1); //OMEGA
                    Matrix yHistory =  Matrix::Zero(4, 1);

					Matrix mPrevIntfVoltage = Matrix::Zero(3, 1);

                    Matrix Dufour_A_k_hat = Matrix::Zero(6, 6);
					Matrix Dufour_B_k_hat = Matrix::Zero(6, 3);
                    Matrix Dufour_B_k_n_hat = Matrix::Zero(6, 3);
					Matrix Jacobian = Matrix::Zero(4, 4);
                    Matrix Dufour_C_k_n = Matrix(3, 6);

					virtual void specificInitialization() override;

                private:
                };    
            }
        }
    }
}