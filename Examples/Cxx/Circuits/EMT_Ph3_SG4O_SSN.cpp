#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS::EMT;

CPS::CIM::Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
CPS::CIM::Examples::Grids::SMIB::ScenarioConfig3 GridParams;
	

void EMT_Ph3_SSN_SG4O_VS()
{
	// Define simulation scenario
	Real timeStep = 0.00001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph3_SG4O_SSN";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);

	// Components
	Matrix param = Matrix::Zero(3, 3);
	param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;

	auto load = CPS::EMT::Ph3::RXLoad::make("Load");
	load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(GridParams.initActivePower/3), 
						CPS::Math::singlePhaseParameterToThreePhase(GridParams.initReactivePower/3),
						GridParams.VnomMV);

	auto sg4o = Ph3::SSN::SG4O::make("SG4O");
	
	sg4o->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, syngenKundur.H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    sg4o->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower, 
							 GridParams.initTerminalVolt);

	// Topology
	load->connect(SimNode::List{ n1 });

	sg4o->connect(SimNode::List{ n1, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{load, sg4o});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I_sg4o_SSN", sg4o->attribute("i_intf"));
	logger->logAttribute("V_sg4o_SSN", sg4o->attribute("v_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.doInitFromNodesAndTerminals(true);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setSolverType(Solver::Type::SSN);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();	
}

void EMT_Ph3_SSN_SG4ODQ_VS()
{

	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph3_SG4O_DQ_SSN";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::ABC);

	// Components
	Matrix param = Matrix::Zero(3, 3);
	param <<
		1., 0, 0,
		0, 1., 0,
		0, 0, 1.;

	auto load = CPS::EMT::Ph3::RXLoad::make("Load");
	load->setParameters(CPS::Math::singlePhaseParameterToThreePhase(GridParams.initActivePower/3), 
						CPS::Math::singlePhaseParameterToThreePhase(GridParams.initReactivePower/3),
						GridParams.VnomMV);

	auto sg4o = Ph3::SSN::SG4O_DQ::make("SG4O_DQ_1");
	
	sg4o->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, syngenKundur.H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    sg4o->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower, 
							 GridParams.initTerminalVolt);

	// Topology
	load->connect(SimNode::List{ n1 });

	sg4o->connect(SimNode::List{ n1, SimNode::GND });

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{load, sg4o});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I_sg4o_SSN", sg4o->attribute("i_intf"));
	logger->logAttribute("V_sg4o_SSN", sg4o->attribute("v_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.doInitFromNodesAndTerminals(true);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setSolverType(Solver::Type::SSN);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();	
}


void test(){
	Matrix t1 = Matrix::Zero(3, 3);
	t1 <<
	0., 2.3, 0.,
	6.3, 5.6, 0.,
	1.3, 0., 5.8;

	Matrix source = Matrix::Zero(3,1);
	source << 3.,4.,5.;

	CPS::SparseMatrixRow sparse = t1.sparseView();
	CPS::LUFactorizedSparse matrixLU;
	matrixLU.analyzePattern(sparse);
	matrixLU.factorize(sparse);
	
	
	Matrix results = Matrix::Zero(3, 1);
	results = matrixLU.solve(source);

	std::cout << t1 << std::endl;
	std::cout << source << std::endl;
	std::cout << results << std::endl;
	std::cout <<"--------------------------------------------------------------------------------" << std::endl;

	//3*e^(4x)-1, 12*e^(4x), 0
	//y*x^4, 4*y*x^3, x^4

	Matrix Jacobian = Matrix::Zero(2, 2);
	Jacobian << 5821982344.92, 0., 3125.0, 625.0; //Start at x=5,y=5

	Matrix newtonSource = Matrix::Zero(2, 1);
	newtonSource << 2., 2.;
	Matrix funcResult = Matrix::Zero(2, 1);
	Matrix error = Matrix::Zero(2, 1);
	Matrix result = Matrix::Zero(2, 1);
	result << 5., 5.;

	int it = 0;
	do
	{

	CPS::SparseMatrixRow newtonSparse = Jacobian.sparseView();
	CPS::LUFactorizedSparse matrixLU;
	matrixLU.analyzePattern(newtonSparse);
	matrixLU.factorize(newtonSparse);
	
	Matrix resultStep = Matrix::Zero(2, 1);

	resultStep = matrixLU.solve(newtonSource-funcResult);		

	result += resultStep;

	funcResult << 3.*expf(4.*result(0,0))-1., result(1, 0)*result(0, 0)*result(0, 0)*result(0, 0)*result(0, 0);

	Jacobian(0, 0) = 12.*expf(4.*result(0,0));
	Jacobian(1, 0) = 4*result(1, 0)*result(0, 0)*result(0, 0)*result(0, 0);
	Jacobian(1,1) = result(0, 0)*result(0, 0)*result(0, 0)*result(0, 0);

	error = funcResult-newtonSource;

	std::cout << Jacobian << std::endl;
	std::cout << error << std::endl;
	std::cout << result << std::endl;

	it++;
	} while (abs(error(0,0)+error(1,0)) > 0.001);
}



void EMT_Ph1_Diode(){
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.1;
	String simName = "EMT_Ph3_SG4O_SSN";
	Logger::setLogDir("logs/" + simName);

	// Nodes
	auto n1 = SimNode::make("n1", PhaseType::Single);
	auto n2 = SimNode::make("n1", PhaseType::Single);

	// Components

	auto vs0 = Ph1::VoltageSource::make("vs0");
	vs0->setParameters(CPS::Complex(1.,0.), 50.0);

	auto load = CPS::EMT::Ph1::Resistor::make("Load");
	load->setParameters(10.);

	auto diode = Ph1::SSN::Diode::make("Diode");
	
	// Topology
	load->connect(SimNode::List{ n1, n2 });

	diode->connect(SimNode::List{ SimNode::GND, n2 });

	vs0->connect(SimNode::List{SimNode::GND, n1});

	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{load, vs0, diode});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("I_Diode_SSN", diode->attribute("i_intf"));
	logger->logAttribute("V_Diode_SSN", diode->attribute("v_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.doInitFromNodesAndTerminals(true);
	sim.setSystem(sys);
	sim.addLogger(logger);
	sim.setDomain(Domain::EMT);
	sim.setSolverType(Solver::Type::SSN);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.run();	
}


int main(){
	//test();
    //EMT_Ph3_SSN_SG4O_VS();
	EMT_Ph3_SSN_SG4ODQ_VS();
	//EMT_Ph1_Diode();
    return 0;
}