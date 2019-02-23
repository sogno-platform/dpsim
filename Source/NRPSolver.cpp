/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "dpsim/NRPSolver.h"

using namespace CPS;
using namespace DPsim;


void NRpolarSolver::resize_sol(int n) {
	sol_P = CPS::Vector(n);
	sol_Q = CPS::Vector(n);
	sol_V = CPS::Vector(n);
	sol_D = CPS::Vector(n);
	sol_P.setZero(n);
	sol_Q.setZero(n);
	sol_V.setZero(n);
	sol_D.setZero(n);
	sol_length = n;
}

void NRpolarSolver::resize_complex_sol(int n) {
	sol_S_complex = CPS::VectorComp(n);
	sol_V_complex = CPS::VectorComp(n);
	sol_S_complex.setZero(n);
	sol_V_complex.setZero(n);
	sol_length = n;
}

/*
* Complex power
*/
CPS::Complex NRpolarSolver::sol_Scx(CPS::UInt k) {
	return CPS::Complex(sol_P.coeff(k), sol_Q.coeff(k));
}

/*
* Real voltage
*/
CPS::Real NRpolarSolver::sol_Vr(UInt k) {
	return sol_V.coeff(k) * cos(sol_D.coeff(k));
}

/*
* Imaginary voltage
*/
CPS::Real NRpolarSolver::sol_Vi(UInt k) {
	return sol_V.coeff(k) * sin(sol_D.coeff(k));
}

/*
* Complex voltage
*/
CPS::Complex NRpolarSolver::sol_Vcx(UInt k) {
	return CPS::Complex(sol_Vr(k), sol_Vi(k));
}



/*
    * constructor
    */
NRpolarSolver::NRpolarSolver(CPS::String simName, CPS::SystemTopology & sysTopology, CPS::Real timeStep, CPS::Domain domain,CPS::Logger::Level logLevel) :
	mLogLevel(logLevel),
	mLog(simName + "_PF", logLevel)
{
	SysTopology = sysTopology;
	mTimeStep = timeStep;

	for (auto comp : SysTopology.mComponents) {
		if (std::shared_ptr<CPS::Static::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::Static::Ph1::SynchronGenerator>(comp))
			SynchronGenerators.push_back(gen);
		else if (std::shared_ptr<CPS::Static::Ph1::Load> load = std::dynamic_pointer_cast<CPS::Static::Ph1::Load>(comp))
			Loads.push_back(load);
		else if (std::shared_ptr<CPS::Static::Ph1::Transformer> trafo = std::dynamic_pointer_cast<CPS::Static::Ph1::Transformer>(comp))
			Transformers.push_back(trafo);
		else if (std::shared_ptr<CPS::Static::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::Static::Ph1::PiLine>(comp))
			Lines.push_back(line);
		else if (std::shared_ptr<CPS::Static::Ph1::externalGridInjection> extnet = std::dynamic_pointer_cast<CPS::Static::Ph1::externalGridInjection>(comp))
			externalGrids.push_back(extnet);
	}

	setSbase();

	compose_Y();
	determinePowerFlowBusType();
}

void NRpolarSolver::NRP_initialize(){

	generate_initial_solution();
	
	mLog.info() << "#### NEWTON-RAPHSON POLAR SOLVER " << std::endl;

	mLog.info() << "#### Admittance Matrix: " <<std::endl
		<< Eigen::Matrix<CPS::Complex,Eigen::Dynamic,Eigen::Dynamic>(Y) << std::endl;

	mLog.info() << "#### Create index vectors for power flow solver:" << std::endl;
	BUSES.reserve(
		PQBusIndices.size() + PVBusIndices.size()
	);
	BUSES.insert(
		BUSES.end(),
		PQBusIndices.begin(),
		PQBusIndices.end());
	BUSES.insert(
		BUSES.end(),
		PVBusIndices.begin(),
		PVBusIndices.end());		
	mLog.info() << "Buses: " << logVector(BUSES) << std::endl;

	PQPV.reserve(
		2 * PQBusIndices.size()
		+ PVBusIndices.size());
	PQPV.insert(
		PQPV.end(),
		PQBusIndices.begin(),
		PQBusIndices.end());
	PQPV.insert(
		PQPV.end(),
		PVBusIndices.begin(),
		PVBusIndices.end());
	mLog.info() << "PQPV: " << logVector(PQPV) << std::endl;

	LastPQ.reserve(PQBusIndices.size());
	LastPQ.insert(
		LastPQ.end(),
		PQBusIndices.begin(),
		PQBusIndices.end());
	mLog.info() << "PQ: " << logVector(LastPQ) << std::endl;

	LastPV.reserve(PVBusIndices.size());
	LastPV.insert(
		LastPV.end(),
		PVBusIndices.begin(),
		PVBusIndices.end());
	mLog.info() << "PV: " << logVector(LastPV) << std::endl;

	/*uInt to int*/
	std::vector<int>slackBusIndex_ = std::vector<int>(slackBusIndex.begin(), slackBusIndex.end());
	mLog.info() << "VD: " << logVector(slackBusIndex_) << std::endl;

	Pesp = sol_P;
	Qesp = sol_Q;

	if (!checks()) {
		throw std::invalid_argument(
			"The grid failed the solver compatibility test.");
	}
}

NRpolarSolver::~NRpolarSolver() noexcept
{
}


bool NRpolarSolver::checks() const
{
    return slackBusIndex.size() <= 1;
}

// Calculate the slack bus power
void NRpolarSolver::calculate_slack_power() {        
    for (auto k: slackBusIndex) {
        CPS::Complex I(0.0, 0.0);
        for (UInt j = 0; j < SysTopology.mNodes.size(); j++) {
            I += Y.coeff(k, j) * sol_Vcx(j);
        }
        I = sol_Vcx(k) * conj(I); //now this is the power
        sol_P(k) = I.real();
        sol_Q(k) = I.imag();
    }
}

// Calculate the reactive power of the bus k (usefull for PV uses)
void NRpolarSolver::calculate_Q(UInt npq, UInt npv) {
    double val;
    UInt k;
    for (UInt i = npq - 1; i < npq + npv; i++) {
        k = PQPV[i];
        val = Q(k);
        sol_Q(k) = val;
    }
}

// Calculate the active power at a bus
double NRpolarSolver::P(UInt k) {
    double val = 0.0;
    for (UInt j = 0; j < SysTopology.mNodes.size(); j++) {
        val += sol_V.coeff(j)
                *(G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j))
                + B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
    }
    return sol_V.coeff(k) * val;
}

// Calculate the reactive power at a bus
double NRpolarSolver::Q(UInt k) {
    double val = 0.0;
    for (UInt j = 0; j < SysTopology.mNodes.size(); j++) {
        val += sol_V.coeff(j)
                *(G(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j))
                - B(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j)));
    }
    return sol_V.coeff(k) * val;
}

// set the parameters for powerflow
void NRpolarSolver::setSbase() {

	CPS::Real maxPower = 0.;
	if (!SynchronGenerators.empty()) {
		for (auto gen : SynchronGenerators)
			if (std::abs(gen->mPV->attribute<CPS::Real>("P_set")->get()) > maxPower)
				maxPower = std::abs(gen->mPV->attribute<CPS::Real>("P_set")->get());
	}
	else if (!Transformers.empty()) {
		for (auto trafo : Transformers)
			if (trafo->attribute<CPS::Real>("S")->get() > maxPower)
				maxPower = trafo->attribute<CPS::Real>("S")->get();
	}
    if (maxPower != 0.)
        Sbase = pow(10, 1 + floor(log10(maxPower)));
	else
	{
		mLog.warn() << "No suitable quantity found for setting Sbase. Using 100kVA." << std::endl;
		Sbase = 100000;
	}			
	mLog.info() << "Base power= " << Sbase << " VA." << std::endl;
}


// set a node to VD using its name
void NRpolarSolver::setVDNode(CPS::String name) {

	switch (externalGrids.empty())
	{
	case false:
		if (externalGrids[0]->node(0)->name() == name) {
			externalGrids[0]->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
			break;
		}
	case true:
		for (auto gen : SynchronGenerators) {
			if (gen->node(0)->name() == name)
			{
				gen->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
				return;
			}
		}
		throw std::invalid_argument("Invalid slack bus, no external grid or synchronous generator attached");
		break;
	}
		
}

void NRpolarSolver::modifyPowerFlowBusComponent(CPS::String name,CPS::PowerflowBusType powerFlowBusType) {
	for (auto comp : SysTopology.mComponents) {
		if (comp->name() == name) {
			if (std::shared_ptr<CPS::Static::Ph1::externalGridInjection> extnet = std::dynamic_pointer_cast<CPS::Static::Ph1::externalGridInjection>(comp))
				extnet->modifyPowerFlowBusType(powerFlowBusType);
			else if(std::shared_ptr<CPS::Static::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::Static::Ph1::SynchronGenerator>(comp))
				gen->modifyPowerFlowBusType(powerFlowBusType);

		}

	}



};

// determines power flow bus type for each node according to the components attached to it.
void NRpolarSolver::determinePowerFlowBusType() {

	// Make sure we do not double-add:
	PQBusIndices.clear();
	PVBusIndices.clear();
	slackBusIndex.clear();

	// Determine powerflow bus types through analysis of system topology
	for (auto node : SysTopology.mNodes) {

		bool connectedPV = false;
		bool connectedPQ = false;
		bool connectedVD = false;

		for (auto comp : SysTopology.mComponentsAtNode[node]) {

			if (std::shared_ptr<CPS::Static::Ph1::Load> load = std::dynamic_pointer_cast<CPS::Static::Ph1::Load>(comp))
			{
				if (load->mPowerflowBusType == CPS::PowerflowBusType::PQ) {
					load->modifyPowerFlowBusType(CPS::PowerflowBusType::PQ);
					connectedPQ = true;
				}
			}
			else if (std::shared_ptr<CPS::Static::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::Static::Ph1::SynchronGenerator>(comp)) {
				    
				if (gen->mPowerflowBusType == CPS::PowerflowBusType::PV) {
					connectedPV = true;
				}
				else if (gen->mPowerflowBusType == CPS::PowerflowBusType::VD) {
					connectedVD = true;
				}

			}
			else if (std::shared_ptr<CPS::Static::Ph1::externalGridInjection> extnet = std::dynamic_pointer_cast<CPS::Static::Ph1::externalGridInjection>(comp)) {
				if (extnet->mPowerflowBusType == CPS::PowerflowBusType::VD) {
					connectedVD = true;
				}
			}
		}
		if (!connectedPV && connectedPQ && !connectedVD) {
			PQBusIndices.push_back(node->simNode());
			PQBuses.push_back(node);
		}
		else if (!connectedPV && !connectedPQ && !connectedVD) {
			PQBusIndices.push_back(node->simNode());
			PQBuses.push_back(node);
		}
		else if (connectedPV && !connectedVD) {
			PVBusIndices.push_back(node->simNode());
			PVBuses.push_back(node);
		}
		else if (!connectedPV && !connectedPQ && connectedVD) {
			slackBusIndex.push_back(node->simNode());
			slackBus.push_back(node);
		}
		else {
			std::stringstream ss;
			ss << "Node>>" << node->name() << ": combination of connected components is invalid";
			throw std::invalid_argument(ss.str());
		}


	}

}



// this could be integrated into the function that determines node type (PV,PQ)
void NRpolarSolver::generate_initial_solution(bool keep_last_solution) {
	resize_sol(SysTopology.mNodes.size());
	resize_complex_sol(SysTopology.mNodes.size());

	for (auto pq : PQBuses) {
		if (!keep_last_solution) {
			sol_V(pq->simNode()) = 1.0;
			sol_D(pq->simNode()) = 0.0;
			sol_V_complex(pq->simNode()) = CPS::Complex(sol_V[pq->simNode()], sol_D[pq->simNode()]);
		}
		for (auto comp : SysTopology.mComponentsAtNode[pq]) {

			if (std::shared_ptr<CPS::Static::Ph1::Load> load = std::dynamic_pointer_cast<CPS::Static::Ph1::Load>(comp)) {
				if (load->use_profile) {
					load->updatePQ();
				}
				sol_P(pq->simNode()) -= load->mPQ->attribute<CPS::Real>("P")->get() / Sbase;
				sol_Q(pq->simNode()) -= load->mPQ->attribute<CPS::Real>("Q")->get() / Sbase;

				sol_S_complex(pq->simNode()) = CPS::Complex(sol_P[pq->simNode()], sol_Q[pq->simNode()]);

			}
		}
	}
	for (auto pv : PVBuses) {
		if (!keep_last_solution) {
			sol_Q(pv->simNode()) = 0;
			sol_D(pv->simNode()) = 0;
		}
		for (auto comp : SysTopology.mComponentsAtNode[pv]) {
			if (std::shared_ptr<CPS::Static::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::Static::Ph1::SynchronGenerator>(comp)) {
				sol_P(pv->simNode()) += gen->mPV->attribute<CPS::Real>("P_set")->get() / Sbase;
				sol_V(pv->simNode()) = gen->mPV->attribute<CPS::Real>("V_set_pu")->get();

			}
			sol_S_complex(pv->simNode()) = CPS::Complex(sol_P[pv->simNode()], sol_Q[pv->simNode()]);
			sol_V_complex(pv->simNode()) = CPS::Complex(sol_V[pv->simNode()], sol_D[pv->simNode()]);

		}
	}
		for (auto vd : slackBus) {
			sol_P(vd->simNode()) = 0.0;
			sol_Q(vd->simNode()) = 0.0;
			if (!SynchronGenerators.empty()) {
				for (auto gen : SynchronGenerators)
				{
                    /// if multiple generators attached to a node,
					///  their voltage should be the same.
					if (gen->node(0)->simNode() == vd->simNode())
						sol_V(vd->simNode()) = gen->mVD->attribute<CPS::Real>("V_set_pu")->get();
				}
			}
			else
				sol_V(vd->simNode()) = 1.0;
			sol_D(vd->simNode()) = 0.0;
			sol_S_complex(vd->simNode()) = CPS::Complex(sol_P[vd->simNode()], sol_Q[vd->simNode()]);
			sol_V_complex(vd->simNode()) = CPS::Complex(sol_V[vd->simNode()], sol_D[vd->simNode()]);
		}
	
		


	sol_initialized = true;
	sol_complex_initialized = true;
	mLog.info() << "#### Initial solution: " << std::endl;
	mLog.info() << "P\t\tQ\t\tV\t\tD" << std::endl;
	for (UInt i = 0; i < sol_length; i++) {
		mLog.info() << sol_P[i] << "\t" << sol_Q[i] << "\t" << sol_V[i] << "\t" << sol_D[i] << std::endl;
	}
}


/*
* Gets the real part of a circuit admittance matrix element
* at row i and column j
*/
double NRpolarSolver::G(int i, int j) {
	//Complex com = (Complex) (Y.coeff(i, j));
	return Y.coeff(i, j).real();
}

/*
* Gets the imaginary part of a circuit admittance matrix element
* at row i and column j
*/
double NRpolarSolver::B(int i, int j) {
	//Complex com = (Complex) (Y.coeff(i, j));
	return Y.coeff(i, j).imag();
}


/*//////////////////////////////////////////////////////////////////////////
    * Calculate the jacobian of the circuit
    */
void NRpolarSolver::Jacobian(Matrix &J, Vector &V, Vector &D, UInt npq, UInt npv) {
    //matrix(rows, cols)
    UInt npqpv = npq + npv;
    double val;
    UInt k, j;
    UInt da, db;

    J.setZero();

    //J1 
    for (UInt a = 0; a < npqpv; a++) { //rows
        k = PQPV[a];
        //diagonal
        J(a, a) = -Q(k) - B(k, k) * V.coeff(k) * V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < npqpv; b++) {
            if (b != a) {
                j = PQPV[b];
                val = V.coeff(k) * V.coeff(j)
                        *(G(k, j) * sin(D.coeff(k) - D.coeff(j))
                        - B(k, j) * cos(D.coeff(k) - D.coeff(j)));
                //if (val != 0.0)
                J(a, b) = val;
            }
        }
    }

    //J2
    da = 0;
    db = npqpv;
    for (UInt a = 0; a < npqpv; a++) { //rows
        k = PQPV[a];
        //diagonal
        //std::cout << "J2D:" << (a + da) << "," << (a + db) << std::endl;
        if (a < npq)
            J(a + da, a + db) = P(k) + G(k, k) * V.coeff(k) * V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < npq; b++) {
            if (b != a) {
                j = PQPV[b];
                val = V.coeff(k) * V.coeff(j)
                        *(G(k, j) * cos(sol_D.coeff(k) - sol_D.coeff(j))
                        + B(k, j) * sin(sol_D.coeff(k) - sol_D.coeff(j)));
                //if (val != 0.0)
                //std::cout << "J2ij:" << (a + da) << "," << (b + db) << std::endl;
                J(a + da, b + db) = val;
            }
        }
    }


    //J3
    da = npqpv;
    db = 0;
    for (UInt a = 0; a < npq; a++) { //rows
        k = PQPV[a];
        //diagonal
        //std::cout << "J3:" << (a + da) << "," << (a + db) << std::endl;
        J(a + da, a + db) = P(k) - G(k, k) * V.coeff(k) * V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < npqpv; b++) {
            if (b != a) {
                j = PQPV[b];
                val = V.coeff(k) * V.coeff(j)
                        *(G(k, j) * cos(D.coeff(k) - D.coeff(j))
                        + B(k, j) * sin(D.coeff(k) - D.coeff(j)));
                //if (val != 0.0)
                //std::cout << "J3:" << (a + da) << "," << (b + db) << std::endl;
                J(a + da, b + db) = -val;
            }
        }
    }

    //J4
    da = npqpv;
    db = npqpv;
    for (UInt a = 0; a < npq; a++) { //rows
        k = PQPV[a];
        //diagonal
        //std::cout << "J4:" << (a + da) << "," << (a + db) << std::endl;
        J(a + da, a + db) = Q(k) - B(k, k) * V.coeff(k) * V.coeff(k);

        //non diagonal elements
        for (UInt b = 0; b < npq; b++) {
            if (b != a) {
                j = PQPV[b];
                val = V.coeff(k) * V.coeff(j)
                        *(G(k, j) * sin(D.coeff(k) - D.coeff(j))
                        - B(k, j) * cos(D.coeff(k) - D.coeff(j)));
                if (val != 0.0) {
                    //std::cout << "J4:" << (a + da) << "," << (b + db) << std::endl;
                    J(a + da, b + db) = val;
                }
            }
        }
    }


}
    
    
/*
     
    def mu(Ybus, J, F, dV, dx, pvpq, pq):
"""
Calculate the Iwamoto acceleration parameter as described in:
"A Load Flow Calculation Method for Ill-Conditioned Power Systems" by Iwamoto, S. and Tamura, Y.
Args:
    Ybus: Admittance matrix
    J: Jacobian matrix
    F: mismatch vector
    dV: voltage increment (in complex form)
    dx: solution vector as calculated dx = solve(J, F)
    pvpq: array of the pq and pv indices
    pq: array of the pq indices

Returns:
    the Iwamoto's optimal multiplier for ill conditioned systems
"""
# evaluate the Jacobian of the voltage derivative
dS_dVm, dS_dVa = dSbus_dV(Ybus, dV)  # compute the derivatives

J11 = dS_dVa[array([pvpq]).T, pvpq].real
J12 = dS_dVm[array([pvpq]).T, pq].real
J21 = dS_dVa[array([pq]).T, pvpq].imag
J22 = dS_dVm[array([pq]).T, pq].imag

# theoretically this is the second derivative matrix
# since the Jacobian has been calculated with dV instead of V
J2 = vstack([
        hstack([J11, J12]),
        hstack([J21, J22])
        ], format="csr")

a = F
b = J * dx
c = 0.5 * dx * J2 * dx

g0 = -a.dot(b)
g1 = b.dot(b) + 2 * a.dot(c)
g2 = -3.0 * b.dot(c)
g3 = 2.0 * c.dot(c)

roots = np.roots([g3, g2, g1, g0])
# three solutions are provided, the first two are complex, only the real solution is valid
return roots[2].real
     
    */


double NRpolarSolver::solve3rdDegreePolynomial(
        double d,
        double c,
        double b,
        double a,
        double x)
        const
{
    double fx = a * x * x * x + b * x * x + c * x + d;
    double fxd = 3.0 * a * x * x + 2.0 * b * x + c;
    double incx = fx / fxd;

    while (abs(incx) > tolerance) {
        x -= incx;
        fx = a * x * x * x + b * x * x + c * x + d;
        fxd = 3.0 * a * x * x + 2.0 * b * x + c;
        incx = fx / fxd;
    }

    return x;
}
    

double NRpolarSolver::mu(Matrix &J, Matrix &J2, Vector &F, Vector &dV, Vector &dD, Vector & dx, UInt npq, UInt npv){
        
                
    Jacobian(J2, dV, dD, npq, npv);               
        
    Vector a = F;
        
    Vector b = J * (dx);        
        
    Vector c(2*npq+npv); //= dx. * b * 0.5;
    for (UInt i=0;i<(2*npq+npv); i++) //this loop is because EIGEN does not want to perform this simple element wise vector multiplication...
        c(i) = dx.coeff(i) * b.coeff(i) * 0.5;
                    
    double g0 = -1* a.dot(b);
    double g1 = b.dot(b) + 2 * a.dot(c);
    double g2 = -3.0 * b.dot(c);
    double g3 = 2.0 * c.dot(c);
           
    double sol = solve3rdDegreePolynomial(g3, g2, g1, g0, 1.0);
    return sol;         
}
    
    

/*//////////////////////////////////////////////////////////////////////////
    * Calculate the power increments
    */
void NRpolarSolver::get_power_inc(Vector& PQinc, UInt npq, UInt npv) {

    UInt npqpv = npq + npv;
    UInt k;
    PQinc.setZero();

    for (UInt a = 0; a < npqpv; a++) {
        //For PQ and PV buses; calculate incP
        k = PQPV[a];
        PQinc(a) = Pesp.coeff(k) - P(k);

        if (a < npq) //only for PQ buses, calculate incQ
            PQinc(a + npqpv) = Qesp.coeff(k) - Q(k);
    }

}


bool NRpolarSolver::converged(const Vector& PQinc, UInt npqpvpq) const
{
    for (UInt k = 0; k < npqpvpq; k++)
        if (abs(PQinc.coeff(k)) > tolerance)
            return false;

    return true;
}
    
    
void NRpolarSolver::get_increments(Vector X, Vector &incV, Vector &incD, UInt npq, UInt npv){
    
    UInt npqpv = npq + npv;
    UInt k;

    for (UInt a = 0; a < npqpv; a++) {
        k = PQPV[a];
        incD(k) = X.coeff(a);

        if (a < npq)
            incV(k) = X.coeff(a + npqpv);
    }
    
}


void NRpolarSolver::update_solution(Vector X, UInt npq, UInt npv) {

    UInt npqpv = npq + npv;
    UInt k;

    for (UInt a = 0; a < npqpv; a++) {
        k = PQPV[a];
        sol_D(k) += X.coeff(a);

        if (a < npq)
            sol_V(k) = sol_V.coeff(k) * (1.0 + X.coeff(a + npqpv));
    }

    //Correct for PV buses
    for (UInt i = npq; i < npq + npv; i++) {
        k = PQPV[i];
        Complex v = sol_Vcx(k);
        // v *= Model.buses[k].v_set_point / abs(v);
        sol_V(k) = abs(v);
        sol_D(k) = arg(v);
    }
}

    
void NRpolarSolver::powerFlow()
{

	UInt npq = PQBusIndices.size();
	UInt npv = PVBusIndices.size();
    UInt npqpvpq = 2 * npq + npv;

    //System : J*X = K
    Matrix J(npqpvpq, npqpvpq);
    Matrix J2(npqpvpq, npqpvpq);
    Vector K(npqpvpq);
    Vector X(npqpvpq);
    Vector incV(sol_length);
    Vector incD(sol_length);
        
    // First shot: Perhaps the model already converged?

    get_power_inc(K, npq, npv);
    auto didConverge = converged(K, npqpvpq);

    Iterations = 0;
    for (unsigned i = 0; i < maxIterations && ! didConverge; ++i) {
        Jacobian(J, sol_V, sol_D, npq, npv);
        Eigen::FullPivLU<Matrix>lu(J); //Full pivot LU
        X = lu.solve(K);
        get_increments(X, incV, incD, npq, npv);
            
        auto mu_ = mu(J, J2, K, incV, incD, X, npq, npv);
            
        //upgrade the solution
        update_solution(X * mu_, npq, npv);

        //Calculate the increment of power for the new iteration
        get_power_inc(K, npq, npv);

        didConverge = converged(K, npqpvpq);
        Iterations = i;

    }
        
    //Calculate the reactive power for the PV buses:
    calculate_Q(npq, npv);	

    if (! didConverge) {
		calculate_slack_power();
		mLog.info() << "Not converged within "<< Iterations<<" iterations." << std::endl;
		mLog.info() << "Result at the last step: " << std::endl;
		mLog.info() << "P\t\tQ\t\tV\t\tD" << std::endl;
		for (UInt i = 0; i < sol_length; i++) {
			mLog.info() << sol_P[i] << "\t" << sol_Q[i] << "\t" << sol_V[i] << "\t" << sol_D[i] << std::endl;
		}


    } else {
		calculate_slack_power();
		mLog.info() << "converged in " << Iterations << " iterations." << std::endl;
		mLog.info() << "Solution: "<<std::endl;
		mLog.info() << "P\t\tQ\t\tV\t\tD" << std::endl;
		for (UInt i = 0; i < sol_length; i++) {
			mLog.info() << sol_P[i] << "\t" << sol_Q[i] << "\t" << sol_V[i] << "\t" << sol_D[i] << std::endl;
		}
    }
}
    
void NRpolarSolver::set_solution() {

    for (UInt i = 0; i < sol_length; i++) {
        sol_S_complex(i) = CPS::Complex(sol_P.coeff(i), sol_Q.coeff(i));
        sol_V_complex(i) = CPS::Complex(sol_V.coeff(i)*cos(sol_D.coeff(i)), sol_V.coeff(i)*sin(sol_D.coeff(i)));
    }

/* update V to each node*/
/* base voltage is based on component */

	for (auto node : SysTopology.mNodes) {
		CPS::Real baseVoltage_;
			
		for (auto comp : SysTopology.mComponentsAtNode[node]) {
			if (std::shared_ptr<CPS::Static::Ph1::Transformer> trans = std::dynamic_pointer_cast<CPS::Static::Ph1::Transformer>(comp)) {
				if (trans->terminal(0)->node()->name() == node->name())
					baseVoltage_ = trans->attribute<CPS::Real>("base_Voltage_End1")->get();
				else if (trans->terminal(1)->node()->name() == node->name()) 
					baseVoltage_ = trans->attribute<CPS::Real>("base_Voltage_End2")->get();
				else
					mLog.info() << "unable to get base voltage at " << node->name() << std::endl;
					
			}
			if (std::shared_ptr<CPS::Static::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::Static::Ph1::PiLine>(comp)) {
				baseVoltage_ = line->attribute<CPS::Real>("base_Voltage")->get();
			}
		}
		std::dynamic_pointer_cast<CPS::Node<CPS::Complex>>(node)->updateVoltage(sol_V_complex(node->simNode())*baseVoltage_);
	}
		calculate_branch_current();
		calculate_branch_flow();
		calculate_nodal_injection();
}
    

/*This function updates the solver solution object power values using the
    * circuit's own solution power values. this is specially usefull when updating
    * the circuit power values while keeping the previous voltage solution
    */
void NRpolarSolver::update_solution_power_from_circuit(){    
    Pesp = sol_P;
    Qesp = sol_Q;
}

/*
* This function composes the circuit admittance matrix
*
* Each circuit branch element has a function to compose its own
* admittance matrix. As each branch element "knows" the indices of the
* busbars where it is connected, it can create an admittance matrix of the
* dimension of the crcuit admittance matrix. If those elemenr Y matrices
* are Sparse, then the mis use of memory is minimal ans the composition
* of the circuit Y matriz becomes super simple: the sum of all the
* branc elements Y_element matrices created as sparse matrices.
*/
void NRpolarSolver::compose_Y() {
	int n = SysTopology.mNodes.size();
	if (n > 0) {
		Y = CPS::SparseMatrixComp(n, n);

		for (auto line : Lines) {
			line->setPerUnitSystem(Sbase, SysTopology.mSystemOmega);
			line->pfApplyAdmittanceMatrixStamp(Y);
		}
		for(auto trans:Transformers) {
			//to check if this transformer could be ignored
			if (trans->attribute("R") == 0 && trans->attribute("L") == 0) {
				mLog.info() << trans->type() << " " << trans->name() << " ignored for R = 0 and L = 0" << std::endl;
				continue;
			}
			trans->setPerUnitSystem(Sbase, SysTopology.mSystemOmega);
			trans->pfApplyAdmittanceMatrixStamp(Y);
		}
		for(auto shunt: Shunts) {
			shunt->setPerUnitSystem(std::abs(shunt->node(0)->initialSingleVoltage()), Sbase, SysTopology.mSystemOmega);
			shunt->pfApplyAdmittanceMatrixStamp(Y);
		}
		    
		}
	if(Lines.empty() && Transformers.empty()) {
		throw std::invalid_argument("There are no bus");
	}
}

void NRpolarSolver::calculate_branch_current() {

	for (auto line : Lines) {
		VectorComp v(2);
		v(0) = sol_V_complex.coeff(line->node(0)->simNode());
		v(1) = sol_V_complex.coeff(line->node(1)->simNode());
		VectorComp current = line->Y_element() *v;
		line->updateCurrent(current.coeff(0));
	}
}

void NRpolarSolver::calculate_branch_flow() {
	for (auto line : Lines) {
		/// S_out=U*conj(I)
		Complex power_out_0 = (sol_V_complex(line->simNode(0)))*std::conj(line->branchCurrentPU());
		line->updateBranchPowerFlow(power_out_0);
	}
}

// this is to store nodal power injection in first line or transformer (in case no line is connected)
// lower level classes (Node, Terminal) can stay unchanged
void NRpolarSolver::calculate_nodal_injection() {

	for (auto node : SysTopology.mNodes) {
		for (auto comp : SysTopology.mComponentsAtNode[node]) {
			if (std::shared_ptr<CPS::Static::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::Static::Ph1::PiLine>(comp)) {
				line->storeNodalInjection(sol_S_complex.coeff(node->simNode()));
				break;
			}
			else if (std::shared_ptr<CPS::Static::Ph1::Transformer> trafo = std::dynamic_pointer_cast<CPS::Static::Ph1::Transformer>(comp)) {
				trafo->storeNodalInjection(sol_S_complex.coeff(node->simNode()));
				break;
			}
		}
	}
}
Real NRpolarSolver::step(Real time) {
	/*
	if switch triggered:
		compose_Y()
	*/
	NRP_initialize();
	powerFlow();
	set_solution();

	return time + mTimeStep;

}

