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
                class SG4O_DQ:
                    public Base::ReducedOrderSynchronGenerator<Real>,
				    public MNAVariableCompInterface,
				    public SharedFactory<SG4O_DQ>,
                    public SSNNonlinearCompInterface
                    {
                    public:
                        /// Defines UID, name, component parameters and logging level
				        SG4O_DQ(String uid, String name, Logger::Level logLevel = Logger::Level::off);
				        /// Defines name and logging level
				        SG4O_DQ(String name, Logger::Level logLevel = Logger::Level::off)
											    : SG4O_DQ(name, name, logLevel) {}

				        SimPowerComp<Real>::Ptr clone(String name);

                        virtual void specificInitialization() override;

        	            virtual void stepInPerUnit() override;

        	            virtual void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
        	            virtual void mnaApplyRightSideVectorStamp(Matrix& rightVector) override;
        	            virtual void mnaPostStep(const Matrix& leftVector) override;

                        void SSNcalculateFunctionResult();
                        void SSNupdateStates();
                        void SSNupdateJacobian();
                        virtual void ssnUpdate(const Matrix& leftVector) override;

                        virtual Bool hasParameterChanged() override {return true;}

                    protected:
                        Matrix States = Matrix::Zero(3, 1); //delta, E_d', E_q'
                        Matrix prevStates = Matrix::Zero(3, 1); //delta, E_d', E_q'
                        Matrix z = Matrix::Zero(1, 1); //omega
                        Matrix prev_z = Matrix::Zero(1, 1);
                        Matrix yHistory = Matrix::Zero(3, 1);
                        Matrix zHistory = Matrix::Zero(1, 1);

                        Real mMechPow = 0.;
                        Real mPrevMechPow = 0.;

                        Real prev_mEf = mEf;
                        
                        Matrix prev_mVdq0 = (**mVdq0);

                        Matrix Jacobian = Matrix::Zero(6, 6);
                    };
            }
        }
    }
}