/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/PFSolverPowerPolar.h>

using namespace DPsim;
using namespace CPS;


PFSolverPowerPolar::PFSolverPowerPolar(CPS::String name, const CPS::SystemTopology &system, CPS::Real timeStep, CPS::Logger::Level logLevel)
    : PFSolver(name, system, timeStep, logLevel){ }

void PFSolverPowerPolar::generateInitialSolution(Real time, bool keep_last_solution) {
	resize_sol(mSystem.mNodes.size());
	resize_complex_sol(mSystem.mNodes.size());

    // update all components for the new time
    for (auto comp : mSystem.mComponents) {
        if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
            if (load->use_profile) {
                load->updatePQ(time);
            }
            load->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
        }
    }

    // set initial solution for the new time
	for (auto pq : mPQBuses) {
		if (!keep_last_solution) {
			sol_V(pq->matrixNodeIndex()) = 1.0;
			sol_D(pq->matrixNodeIndex()) = 0.0;
			sol_V_complex(pq->matrixNodeIndex()) = CPS::Complex(sol_V[pq->matrixNodeIndex()], sol_D[pq->matrixNodeIndex()]);
		}
		for (auto comp : mSystem.mComponentsAtNode[pq]) {
            if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
                sol_P(pq->matrixNodeIndex()) -= load->attribute<CPS::Real>("P_pu")->get();
                sol_Q(pq->matrixNodeIndex()) -= load->attribute<CPS::Real>("Q_pu")->get();
            }
            else if(std::shared_ptr<CPS::SP::Ph1::SolidStateTransformer> sst =
                std::dynamic_pointer_cast<CPS::SP::Ph1::SolidStateTransformer>(comp)){
                    sol_P(pq->matrixNodeIndex()) -= sst->getNodalInjection(pq).real();
                    sol_Q(pq->matrixNodeIndex()) -= sst->getNodalInjection(pq).imag();
            }
            else if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi =
				std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
                // TODO: add per-unit attributes to VSI and use here
				sol_P(pq->matrixNodeIndex()) += vsi->attribute<CPS::Real>("P_ref")->get() / mBaseApparentPower;
				sol_Q(pq->matrixNodeIndex()) += vsi->attribute<CPS::Real>("Q_ref")->get() / mBaseApparentPower;
			}
            sol_S_complex(pq->matrixNodeIndex()) = CPS::Complex(sol_P[pq->matrixNodeIndex()], sol_Q[pq->matrixNodeIndex()]);
		}
	}

	for (auto pv : mPVBuses) {
		if (!keep_last_solution) {
			sol_Q(pv->matrixNodeIndex()) = 0;
			sol_D(pv->matrixNodeIndex()) = 0;
		}
		for (auto comp : mSystem.mComponentsAtNode[pv]) {
			if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
				sol_P(pv->matrixNodeIndex()) += gen->attribute<CPS::Real>("P_set_pu")->get();
				sol_V(pv->matrixNodeIndex()) = gen->attribute<CPS::Real>("V_set_pu")->get();
			}
            else if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
				sol_P(pv->matrixNodeIndex()) -= load->attribute<CPS::Real>("P_pu")->get();
			}
            else if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi =
				std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
				sol_P(pv->matrixNodeIndex()) += vsi->attribute<CPS::Real>("P_ref")->get() / mBaseApparentPower;
			}
            else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet =
				std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
				sol_P(pv->matrixNodeIndex()) += extnet->attribute<CPS::Real>("p_inj")->get() / mBaseApparentPower;
				sol_V(pv->matrixNodeIndex()) = extnet->attribute<CPS::Real>("V_set_pu")->get();
			}
			sol_S_complex(pv->matrixNodeIndex()) = CPS::Complex(sol_P[pv->matrixNodeIndex()], sol_Q[pv->matrixNodeIndex()]);
			sol_V_complex(pv->matrixNodeIndex()) = CPS::Complex(sol_V[pv->matrixNodeIndex()], sol_D[pv->matrixNodeIndex()]);
		}
	}

    for (auto vd : mVDBuses) {
        sol_P(vd->matrixNodeIndex()) = 0.0;
        sol_Q(vd->matrixNodeIndex()) = 0.0;
        sol_V(vd->matrixNodeIndex()) = 1.0;
        sol_D(vd->matrixNodeIndex()) = 0.0;

        // if external injection at VD bus, reset the voltage to injection's voltage set-point
        for (auto comp : mSystem.mComponentsAtNode[vd]) {
            if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet = std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
                sol_V(vd->matrixNodeIndex()) = extnet->attribute<CPS::Real>("V_set_pu")->get();
            }
        }

        // if generator at VD bus, reset the voltage to generator's set-point
        if (!mSynchronGenerators.empty()) {
            for (auto gen : mSynchronGenerators)
            {
                if (gen->node(0)->matrixNodeIndex() == vd->matrixNodeIndex())
                    sol_V(vd->matrixNodeIndex()) = gen->attribute<CPS::Real>("V_set_pu")->get();
            }
        }

        sol_S_complex(vd->matrixNodeIndex()) = CPS::Complex(sol_P[vd->matrixNodeIndex()], sol_Q[vd->matrixNodeIndex()]);
        sol_V_complex(vd->matrixNodeIndex()) = CPS::Complex(sol_V[vd->matrixNodeIndex()], sol_D[vd->matrixNodeIndex()]);
    }

	solutionInitialized = true;
	solutionComplexInitialized = true;

    Pesp = sol_P;
    Qesp = sol_Q;

	mSLog->info("#### Initial solution: ");
	mSLog->info("P\t\tQ\t\tV\t\tD");
	for (UInt i = 0; i < mSystem.mNodes.size(); ++i) {
		mSLog->info("{}\t{}\t{}\t{}", sol_P[i], sol_Q[i], sol_V[i], sol_D[i]);
	}
    mSLog->flush();
}

void PFSolverPowerPolar::calculateMismatch() {
    UInt npqpv = mNumPQBuses+mNumPVBuses;
    UInt k;
    mF.setZero();

    for (UInt a = 0; a < npqpv; ++a) {
        // For PQ and PV buses calculate active power mismatch
        k = mPQPVBusIndices[a];
        mF(a) = Pesp.coeff(k) - P(k);

        //only for PQ buses calculate reactive power mismatch
        if (a < mNumPQBuses)
            mF(a + npqpv) = Qesp.coeff(k) - Q(k);
    }
}

void PFSolverPowerPolar::calculateJacobian() {
    UInt npqpv = mNumPQBuses + mNumPVBuses;
    double val;
    UInt k, j;
    UInt da, db;

    mJ.setZero();

    //J1
    for (UInt a = 0; a < npqpv; ++a) { //rows
        k = mPQPVBusIndices[a];
        //diagonal
        mJ.coeffRef(a, a) = -Q(k) - B(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < npqpv; ++b) {
            if (b != a) {
                j = mPQPVBusIndices[b];
                val = sol_V.coeff(k) * sol_V.coeff(j)
                        *(G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j))
                        - B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
                //if (val != 0.0)
                mJ.coeffRef(a, b) = val;
            }
        }
    }

    //J2
    da = 0;
    db = npqpv;
    for (UInt a = 0; a < npqpv; ++a) { //rows
        k = mPQPVBusIndices[a];
        //diagonal
        //std::cout << "J2D:" << (a + da) << "," << (a + db) << std::endl;
        if (a < mNumPQBuses)
            mJ.coeffRef(a + da, a + db) = P(k) + G(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < mNumPQBuses; ++b) {
            if (b != a) {
                j = mPQPVBusIndices[b];
                val = sol_V.coeff(k) * sol_V.coeff(j)
                        *(G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j))
                        + B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
                //if (val != 0.0)
                //std::cout << "J2ij:" << (a + da) << "," << (b + db) << std::endl;
                mJ.coeffRef(a + da, b + db) = val;
            }
        }
    }


    //J3
    da = npqpv;
    db = 0;
    for (UInt a = 0; a < mNumPQBuses; ++a) { //rows
        k = mPQPVBusIndices[a];
        //diagonal
        //std::cout << "J3:" << (a + da) << "," << (a + db) << std::endl;
        mJ.coeffRef(a + da, a + db) = P(k) - G(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < npqpv; ++b) {
            if (b != a) {
                j = mPQPVBusIndices[b];
                val = sol_V.coeff(k) * sol_V.coeff(j)
                        *(G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j))
                        + B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
                //if (val != 0.0)
                //std::cout << "J3:" << (a + da) << "," << (b + db) << std::endl;
                mJ.coeffRef(a + da, b + db) = -val;
            }
        }
    }

    //J4
    da = npqpv;
    db = npqpv;
    for (UInt a = 0; a < mNumPQBuses; ++a) { //rows
        k = mPQPVBusIndices[a];
        //diagonal
        //std::cout << "J4:" << (a + da) << "," << (a + db) << std::endl;
        mJ.coeffRef(a + da, a + db) = Q(k) - B(k, k) * sol_V.coeff(k) * sol_V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < mNumPQBuses; ++b) {
            if (b != a) {
                j = mPQPVBusIndices[b];
                val = sol_V.coeff(k) * sol_V.coeff(j)
                        *(G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j))
                        - B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
                if (val != 0.0) {
                    //std::cout << "J4:" << (a + da) << "," << (b + db) << std::endl;
                    mJ.coeffRef(a + da, b + db) = val;
                }
            }
        }
    }
}

void PFSolverPowerPolar::updateSolution() {
    UInt npqpv = mNumPQBuses + mNumPVBuses;
    UInt k;

    for (UInt a = 0; a < npqpv; ++a) {
        k = mPQPVBusIndices[a];
        sol_D(k) += mX.coeff(a);

        if (a < mNumPQBuses)
            sol_V(k) = sol_V.coeff(k) * (1.0 + mX.coeff(a + npqpv));
    }

    //Correct for PV buses
    for (UInt i = mNumPQBuses; i < npqpv; ++i) {
        k = mPQPVBusIndices[i];
        Complex v = sol_Vcx(k);
        // v *= Model.buses[k].v_set_point / abs(v);
        sol_V(k) = abs(v);
        sol_D(k) = arg(v);
    }
}

void PFSolverPowerPolar::setSolution() {
    if (! isConverged) {
		mSLog->info("Not converged within {} iterations", mIterations);
    }
	else {
		calculatePAndQAtSlackBus();
        calculateQAtPVBuses();
		mSLog->info("converged in {} iterations",mIterations);
		mSLog->info("Solution: ");
		mSLog->info("P\t\tQ\t\tV\t\tD");
		for (UInt i = 0; i < mSystem.mNodes.size(); ++i) {
			mSLog->info("{}\t{}\t{}\t{}", sol_P[i], sol_Q[i], sol_V[i], sol_D[i]);
		}
    }
    for (UInt i = 0; i < mSystem.mNodes.size(); ++i) {
        sol_S_complex(i) = CPS::Complex(sol_P.coeff(i), sol_Q.coeff(i));
        sol_V_complex(i) = CPS::Complex(sol_V.coeff(i)*cos(sol_D.coeff(i)), sol_V.coeff(i)*sin(sol_D.coeff(i)));
    }

    /* update V to each node*/
    /* base voltage is based on component */
	for (auto node : mSystem.mNodes) {
		CPS::Real baseVoltage_ = 0;
		for (auto comp : mSystem.mComponentsAtNode[node]) {
            if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi = std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)) {
				baseVoltage_=Math::abs(vsi->attribute<CPS::Complex>("vnom")->get());
				break;
			}
            else if (std::shared_ptr<CPS::SP::Ph1::RXLine> rxline = std::dynamic_pointer_cast<CPS::SP::Ph1::RXLine>(comp)) {
				baseVoltage_ = rxline->attribute<CPS::Real>("base_Voltage")->get();
                break;
			}
            else if (std::shared_ptr<CPS::SP::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp)) {
				baseVoltage_ = line->attribute<CPS::Real>("base_Voltage")->get();
                break;
			}
			else if (std::shared_ptr<CPS::SP::Ph1::Transformer> trans = std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp)) {
				if (trans->terminal(0)->node()->name() == node->name()){
                    baseVoltage_ = trans->attribute<CPS::Real>("nominal_voltage_end1")->get();
                    break;
                }
				else if (trans->terminal(1)->node()->name() == node->name()){
                    baseVoltage_ = trans->attribute<CPS::Real>("nominal_voltage_end2")->get();
                    break;
                }
            else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
                    baseVoltage_ =gen->attribute<CPS::Real>("base_Voltage")->get();
                    break;
                }
            else
                mSLog->warn("Unable to get base voltage at {}", node->name());
            }
        }
		std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(node)->setVoltage(sol_V_complex(node->matrixNodeIndex())*baseVoltage_);
    }
    calculateBranchFlow();
    calculateNodalInjection();
}

void PFSolverPowerPolar::calculateBranchFlow() {
	for (auto line : mLines) {
		VectorComp v(2);
		v(0) = sol_V_complex.coeff(line->node(0)->matrixNodeIndex());
		v(1) = sol_V_complex.coeff(line->node(1)->matrixNodeIndex());
		/// I = Y * V
		VectorComp current = line->Y_element() * v;
		/// pf on branch [S_01; S_10] = [V_0 * conj(I_0); V_1 * conj(I_1)]
		VectorComp flow_on_branch = v.array()*current.conjugate().array();
		line->updateBranchFlow(current,flow_on_branch);
	}
	for (auto trafo : mTransformers) {
		VectorComp v(2);
		v(0) = sol_V_complex.coeff(trafo->node(0)->matrixNodeIndex());
		v(1) = sol_V_complex.coeff(trafo->node(1)->matrixNodeIndex());
		/// I = Y * V
		VectorComp current = trafo->Y_element() * v;
		/// pf on branch [S_01; S_10] = [V_0 * conj(I_0); V_1 * conj(I_1)]
		VectorComp flow_on_branch = v.array()*current.conjugate().array();
		trafo->updateBranchFlow(current, flow_on_branch);
	}
}

void PFSolverPowerPolar::calculateNodalInjection() {
	for (auto node : mSystem.mNodes) {
		std::list<std::shared_ptr<CPS::SP::Ph1::PiLine>> lines;
		for (auto comp : mSystem.mComponentsAtNode[node]) {
			if (std::shared_ptr<CPS::SP::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp)) {
				line->storeNodalInjection(sol_S_complex.coeff(node->matrixNodeIndex()));
				lines.push_back(line);
				break;
			}
		}
		if (lines.empty()) {
			for (auto comp : mSystem.mComponentsAtNode[node]) {
				if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo = std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp)) {
					trafo->storeNodalInjection(sol_S_complex.coeff(node->matrixNodeIndex()));
					break;
				}
			}
		}
	}
}

Real PFSolverPowerPolar::P(UInt k) {
    Real val = 0.0;
    for (UInt j = 0; j < mSystem.mNodes.size(); ++j) {
        val += sol_V.coeff(j)
                *(G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j))
                + B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
    }
    return sol_V.coeff(k) * val;
}

Real PFSolverPowerPolar::Q(UInt k) {
    Real val = 0.0;
    for (UInt j = 0; j < mSystem.mNodes.size(); ++j) {
        val += sol_V.coeff(j)
                *(G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j))
                - B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
    }
    return sol_V.coeff(k) * val;
}

void PFSolverPowerPolar::calculatePAndQAtSlackBus() {
    for (auto k: mVDBusIndices) {
        CPS::Complex I(0.0, 0.0);
        for (UInt j = 0; j < mSystem.mNodes.size(); ++j) {
            I += mY.coeff(k, j) * sol_Vcx(j);
        }
        CPS::Complex S(0.0, 0.0);
        S = sol_Vcx(k) * conj(I);
        sol_P(k) = S.real();
        sol_Q(k) = S.imag();
        for(auto extnet : mExternalGrids){
            if(extnet->mPowerflowBusType==CPS::PowerflowBusType::VD){
			    extnet->updatePowerInjection(S*mBaseApparentPower);
                break;
            }
        }
        for(auto gen : mSynchronGenerators){
            if(gen->mPowerflowBusType==CPS::PowerflowBusType::VD){
			    gen->updatePowerInjection(S*mBaseApparentPower);
                break;
            } 
        }
    }
}

void PFSolverPowerPolar::calculateQAtPVBuses() {
        for (auto k: mPVBusIndices) {
        CPS::Complex I(0.0, 0.0);
        for (UInt j = 0; j < mSystem.mNodes.size(); ++j) {
            I += mY.coeff(k, j) * sol_Vcx(j);
        }
        CPS::Complex S(0.0, 0.0);
        S = sol_Vcx(k) * conj(I);
        // sol_P(k) = S.real();
        sol_Q(k) = S.imag();
        for(auto gen : mSynchronGenerators){
            if(gen->mPowerflowBusType==CPS::PowerflowBusType::PV){
			    gen->updateReactivePowerInjection(S*mBaseApparentPower);
                break;
            }        
        }
    }
}

void PFSolverPowerPolar::resize_sol(Int n) {
	sol_P = CPS::Vector(n);
	sol_Q = CPS::Vector(n);
	sol_V = CPS::Vector(n);
	sol_D = CPS::Vector(n);
	sol_P.setZero(n);
	sol_Q.setZero(n);
	sol_V.setZero(n);
	sol_D.setZero(n);
}

void PFSolverPowerPolar::resize_complex_sol(Int n) {
	sol_S_complex = CPS::VectorComp(n);
	sol_V_complex = CPS::VectorComp(n);
	sol_S_complex.setZero(n);
	sol_V_complex.setZero(n);
}

CPS::Real PFSolverPowerPolar::sol_Vr(UInt k) {
	return sol_V.coeff(k) * cos(sol_D.coeff(k));
}

CPS::Real PFSolverPowerPolar::sol_Vi(UInt k) {
	return sol_V.coeff(k) * sin(sol_D.coeff(k));
}

CPS::Complex PFSolverPowerPolar::sol_Vcx(UInt k) {
	return CPS::Complex(sol_Vr(k), sol_Vi(k));
}
