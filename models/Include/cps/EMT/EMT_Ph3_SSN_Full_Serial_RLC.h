
#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/Base/Base_Ph3_Inductor.h>
#include <cps/Base/Base_Ph3_Capacitor.h>
#include <cps/Base/Base_Ph3_Resistor.h>

namespace CPS{
    namespace EMT{
        namespace Ph3{
            namespace SSN{
                class Full_Serial_RLC:
				    public MNAInterface,
				    public SimPowerComp<Real>,
				    public SharedFactory<Full_Serial_RLC>,
					public Base::Ph3::Resistor,
					public Base::Ph3::Inductor,
					public Base::Ph3::Capacitor
                    {
                public:
                    /// Defines UID, name, component parameters and logging level
				    Full_Serial_RLC(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				    /// Defines name and logging level
				    Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
											    : Full_Serial_RLC(name, name, logLevel) {}

				    SimPowerComp<Real>::Ptr clone(String name);
                    void setParameters(Matrix resistance, Matrix inductance, Matrix capacitance);

				    // #### General ####
				    /// Initializes component from power flow data
				    void initializeFromNodesAndTerminals(Real frequency);

			    	// #### MNA section ####
				    /// Initializes internal variables of the component
				    void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
				    /// Stamps system matrix
				    void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
				    /// Stamps right side (source) vector
				    void mnaApplyRightSideVectorStamp(Matrix& rightVector);
				    /// Update interface voltage from MNA system result
				    void mnaUpdateVoltage(const Matrix& leftVector);
				    /// Update interface current from MNA system result
				    void mnaUpdateCurrent(const Matrix& leftVector);
				    /// MNA pre step operations
				    void mnaPreStep(Real time, Int timeStepCount);
				    /// MNA post step operations
				    void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
				    /// Add MNA pre step dependencies
				    void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
				    /// Add MNA post step dependencies
				    void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

					void ssnUpdateState();

				    class MnaPreStep : public Task {
				    public:
					    MnaPreStep(Full_Serial_RLC& Full_Serial_RLC) :
						    Task(**Full_Serial_RLC.mName + ".MnaPreStep"), mFull_Serial_RLC(Full_Serial_RLC) {
							    mFull_Serial_RLC.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
					    }
					    void execute(Real time, Int timeStepCount) { mFull_Serial_RLC.mnaPreStep(time, timeStepCount); };
				    private:
					    Full_Serial_RLC& mFull_Serial_RLC;
				    };

				    class MnaPostStep : public Task {
				    public:
					    MnaPostStep(Full_Serial_RLC& full_Serial_RLC, Attribute<Matrix>::Ptr leftVector) :
					    	Task(**full_Serial_RLC.mName + ".MnaPostStep"),
						    mFull_Serial_RLC(full_Serial_RLC), mLeftVector(leftVector) {
						    	mFull_Serial_RLC.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
					    }
					    void execute(Real time, Int timeStepCount) { mFull_Serial_RLC.mnaPostStep(time, timeStepCount, mLeftVector); };
				    private:
				    	Full_Serial_RLC& mFull_Serial_RLC;
				    	Attribute<Matrix>::Ptr mLeftVector;
                    };
                protected:
                    Matrix State = Matrix::Zero(6, 1);
                    Matrix yHistory =  Matrix::Zero(3, 1);

					Matrix Dufour_u_n_t = Matrix::Zero(3, 1);

                    Matrix Dufour_A_k_hat = Matrix::Zero(6, 6);
					Matrix Dufour_B_k_hat = Matrix::Zero(6, 3);
                    Matrix Dufour_B_k_n_hat = Matrix::Zero(6, 3);
					Matrix Dufour_W_k_n = Matrix::Zero(3, 3);
                    Matrix Dufour_C_k_n = Matrix(3, 6);
                private:
                };    
            }
        }
    }
}