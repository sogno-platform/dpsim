/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
Examples::Grids::SMIB::ReducedOrderSynchronGenerator::Scenario4::GridParams GridParams;

// Generator parameters
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

void EMT_3ph_SynGen_Fault(String simName, Real timeStep, Real finalTime, Real logDownSampling, Logger::Level logLevel) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	// Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, GridParams.VnomMV, GridParams.setPointActivePower, 
						 GridParams.setPointVoltage, PowerflowBusType::PV);
    genPF->setBaseVoltage(GridParams.VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	// Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(GridParams.VnomMV);
	extnetPF->setBaseVoltage(GridParams.VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	// Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(GridParams.lineResistance, GridParams.lineInductance, 
						  GridParams.lineCapacitance, GridParams.lineConductance);
	linePF->setBaseVoltage(GridParams.VnomMV);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
	loggerPF->logAttribute("i_slack", extnetPF->attribute("i_intf"));
	loggerPF->logAttribute("p_slack", extnetPF->attribute("p_inj"));
	loggerPF->logAttribute("q_slack", extnetPF->attribute("q_inj"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(0.1);
	simPF.setFinalTime(0.2);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();


	// ----- Dynamic simulation ------
	String simNameEMT = simName;
	Logger::setLogDir("logs/"+simNameEMT);
	
	// absolute tolerances
	Real voltage_absTolerance = 1e-6;
	Real current_absTolerance = 1e-6;
	Real sg_absTolerance = 1e-6;

	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Complex initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0), 
											n1PF->voltage()(0,0) * SHIFT_TO_PHASE_B,
											n1PF->voltage()(0,0) * SHIFT_TO_PHASE_C
										  };
	auto n1EMT = SimNode<Real>::make("n1EMT", PhaseType::ABC, initialVoltage_n1);
	n1EMT->daeSetAbsoluteTolerance(voltage_absTolerance);

	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0), 
											n2PF->voltage()(0,0) * SHIFT_TO_PHASE_B,
											n2PF->voltage()(0,0) * SHIFT_TO_PHASE_C
										  };
	auto n2EMT = SimNode<Real>::make("n2EMT", PhaseType::ABC, initialVoltage_n2);
	n2EMT->daeSetAbsoluteTolerance(voltage_absTolerance);

	// Synchronous generator
	auto genEMT = EMT::Ph3::SynchronGenerator3OrderVBR::make("SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, syngenKundur.H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Td0_t);
	/*
	auto genEMT = EMT::Ph3::SynchronGenerator4OrderVBR::make("SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, syngenKundur.H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, 
			syngenKundur.Td0_t, syngenKundur.Tq0_t); 
	*/
    genEMT->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genEMT->daeSetAbsoluteTolerance(sg_absTolerance);
	
	//Grid bus as Slack
	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", logLevel);
	extnetEMT->daeSetAbsoluteTolerance(voltage_absTolerance);

    // Line
	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", logLevel);
	lineEMT->setParameters(Math::singlePhaseParameterToThreePhase(GridParams.lineResistance), 
	                      Math::singlePhaseParameterToThreePhase(GridParams.lineInductance), 
					      Math::singlePhaseParameterToThreePhase(GridParams.lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(GridParams.lineConductance));
	lineEMT->daeSetAbsoluteTolerance(current_absTolerance);

	// Topology
	genEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	extnetEMT->connect({ n2EMT });

	auto systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{genEMT, lineEMT, extnetEMT});

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	loggerEMT->logAttribute("i_slack", 	extnetEMT->attribute("i_intf"));
	loggerEMT->logAttribute("i_line", 	lineEMT->attribute("i_intf"));
	loggerEMT->logAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->logAttribute("Te", 		genEMT->attribute("Te"));
    loggerEMT->logAttribute("Theta", 	genEMT->attribute("Theta"));
    loggerEMT->logAttribute("w_r", 		genEMT->attribute("w_r"));
	loggerEMT->logAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	loggerEMT->logAttribute("Idq0", 	genEMT->attribute("Idq0"));
	loggerEMT->logAttribute("Edq0", 	genEMT->attribute("Edq0_t"));

	
	//Real initActivePower_slack = simPF.getIdObjAttribute("Slack", "p_inj")->get();
	//Real initReactivePower_slack = simPF.getIdObjAttribute("Slack", "q_inj");
	//Complex initElecPower = Complex(initActivePower_slack, initReactivePower_slack);
	Complex initElecPower_slack = Complex(-242575924.156291, 4229197.425543);
	std::cout << "initElecPower = " << initElecPower_slack << std::endl;

	// Set initial derivative of slack current
	Complex I = std::conj(initElecPower_slack / Complex(24000., 0)) * sqrt(2./3.);
	extnetEMT->setInitialComplexIntfCurrent(-I);

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setDomain(Domain::EMT);
	simEMT.setSolverType(Solver::Type::DAE);
	simEMT.setVariableStepSize(false);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setRelativeTolerance(1e-10);
	simEMT.doSplitSubnets(false);
	simEMT.addLogger(loggerEMT);
	//simEMT.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	//simEMT.doSystemMatrixRecomputation(true);
	simEMT.run();
}

int main(int argc, char* argv[]) {	

	//Simultion parameters
	Real finalTime = 0.1;
	Real timeStep = 100e-6;

	Real logDownSampling;
	if (timeStep<1e-6)
		logDownSampling = floor((10e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::debug;

	std::string simName ="DAE_SynGen_SMIB";
	EMT_3ph_SynGen_Fault(simName, timeStep, finalTime, logDownSampling, logLevel);
}