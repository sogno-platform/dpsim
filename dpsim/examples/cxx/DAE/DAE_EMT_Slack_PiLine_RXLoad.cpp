/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

int main(int argc, char* argv[]) {
	// Parameters
	Real frequency = 50;
	Real Vnom = 110e3;

    // PiLine parameters
	Real line_resistance = 0.05;
	Real line_inductance = 0.1;
	Real line_capacitance = 0.1e-6;
	Real line_conductance = 1.0e-6;
	Matrix line_rparam = Matrix::Zero(3, 3);
    Matrix line_lparam = Matrix::Zero(3, 3);
	Matrix line_cparam = Matrix::Zero(3, 3);
	Matrix line_gparam = Matrix::Zero(3, 3);
	line_rparam <<
		line_resistance, 0, 0,
		0, line_resistance, 0,
		0, 0, line_resistance;
	line_lparam <<
		line_inductance, 0, 0,
		0, line_inductance, 0,
		0, 0, line_inductance;
	line_cparam <<
		line_capacitance, 0, 0,
		0, line_capacitance, 0,
		0, 0, line_capacitance;
	line_gparam <<
		line_conductance, 0, 0,
		0, line_conductance, 0,
		0, 0, line_conductance;

    // RXLoad Parameters
	Real pLoadNom = 0.5e6;
	Real qLoadNom = 0.5e6;
	Real load_resistance = std::pow(Vnom/sqrt(3), 2) * (1/pLoadNom);
	Real load_inductance = std::pow(Vnom/sqrt(3), 2) * (1/qLoadNom) / (2 * PI * frequency);
	Matrix p_load = Matrix::Zero(3, 3);
	p_load <<
		pLoadNom, 0, 0,
		0, pLoadNom, 0,
		0, 0, pLoadNom;
	Matrix q_load = Matrix::Zero(3, 3);
	q_load <<
		qLoadNom, 0, 0,
		0, qLoadNom, 0,
		0, 0, qLoadNom;
	Matrix rload_param = Matrix::Zero(3, 3);
    Matrix iload_param = Matrix::Zero(3, 3);
	rload_param <<
		load_resistance, 0, 0,
		0, load_resistance, 0,
		0, 0, load_resistance;
	iload_param <<
		load_inductance, 0, 0,
		0, load_inductance, 0,
		0, 0, load_inductance;
	
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = 0.5;
	Real finalTimePF = 1;
	String simNamePF = "SP_Slack_PiLine_RxLoad_Init";
	Logger::setLogDir("logs/" + simNamePF);

	// Nodes
	auto n1PF = SimNode<Complex>::make("n1PF", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2PF", PhaseType::Single);
    auto n3PF = SimNode<Complex>::make("n3PF", PhaseType::Single);

	// Components
	// voltage source
	auto vsPF = SP::Ph1::VoltageSource::make("vsPF");
	vsPF->setParameters(Vnom*RMS3PH_TO_PEAK1PH);

	// PiLine
	auto PiLine_conductance0PF = SP::Ph1::Resistor::make("con0_linePF", Logger::Level::info);
	PiLine_conductance0PF->setParameters(2/line_conductance);
	auto PiLine_conductance1PF = SP::Ph1::Resistor::make("con1_linePF");
	PiLine_conductance1PF->setParameters(2/line_conductance);
	auto PiLine_capacitance0PF = SP::Ph1::Capacitor::make("cap0_linePF");
	PiLine_capacitance0PF->setParameters(line_capacitance/2);
	auto PiLine_capacitance1PF = SP::Ph1::Capacitor::make("cap1_linePF");
	PiLine_capacitance1PF->setParameters(line_capacitance/2);
	auto PiLine_resistancePF = SP::Ph1::Resistor::make("res_linePF");
	PiLine_resistancePF->setParameters(line_resistance);
    auto PiLine_inductancePF = SP::Ph1::Inductor::make("ind_linePF");
	PiLine_inductancePF->setParameters(line_inductance);

	// RXLoad
    auto rloadPF = SP::Ph1::Resistor::make("rloadPF");
	rloadPF->setParameters(load_resistance);
	auto iloadPF = SP::Ph1::Inductor::make("iloadPF");
	iloadPF->setParameters(load_inductance);

	// Topology
	vsPF->connect({SimNode<Complex>::GND, n1PF});
	PiLine_conductance0PF->connect({SimNode<Complex>::GND, n1PF});
	PiLine_capacitance0PF->connect({SimNode<Complex>::GND, n1PF});
    PiLine_inductancePF->connect({n2PF, n1PF});
	PiLine_resistancePF->connect({n3PF, n2PF});
	PiLine_conductance1PF->connect({SimNode<Complex>::GND, n3PF});
	PiLine_capacitance1PF->connect({SimNode<Complex>::GND, n3PF});
	rloadPF->connect({ SimNode<Complex>::GND, n3PF });
	iloadPF->connect({ SimNode<Complex>::GND, n3PF });

	auto systemPF  = SystemTopology(frequency, SystemNodeList{ n1PF, n2PF, n3PF}, 
		SystemComponentList{ vsPF, PiLine_conductance0PF, PiLine_capacitance0PF, 
			PiLine_inductancePF, PiLine_resistancePF, rloadPF, iloadPF,
			PiLine_conductance1PF, PiLine_capacitance1PF});

	// Logging
	auto loggerPF  = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
    loggerPF->logAttribute("v3", n3PF->attribute("v"));
	loggerPF->logAttribute("i_slack", vsPF->attribute("i_intf"));
	loggerPF->logAttribute("i_line", PiLine_inductancePF->attribute("i_intf"));
	loggerPF->logAttribute("v_line", PiLine_inductancePF->attribute("v_intf"));
	loggerPF->logAttribute("i_cap0", PiLine_capacitance0PF->attribute("i_intf"));
	loggerPF->logAttribute("i_con0", PiLine_conductance0PF->attribute("i_intf"));
	loggerPF->logAttribute("i_cap1", PiLine_capacitance1PF->attribute("i_intf"));
	loggerPF->logAttribute("i_con1", PiLine_conductance1PF->attribute("i_intf"));

	Simulation simPF(simNamePF, Logger::Level::info);
	simPF.setSystem(systemPF);
	simPF.setDomain(Domain::SP);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.addLogger(loggerPF);
	simPF.run();


	// ----- DYNAMIC SIMULATION -----
	Real timeStepEMT  = 0.0001;
	Real finalTimeEMT = 1;
	String simNameEMT = "DAE_EMT_Slack_PiLine_RXLoad";
	Logger::setLogDir("logs/" + simNameEMT);
	Real voltage_absTolerance = Vnom*0.00001;		//0.001% = aprox. 1V
	Real current_absTolerance = sqrt(pLoadNom*pLoadNom+qLoadNom*qLoadNom)/Vnom*0.00001;

	// Nodes
    // Node n1
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0)*PEAK1PH_TO_RMS3PH, 
											n1PF->voltage()(0,0)*SHIFT_TO_PHASE_B*PEAK1PH_TO_RMS3PH,
											n1PF->voltage()(0,0)*SHIFT_TO_PHASE_C*PEAK1PH_TO_RMS3PH
										  };
	auto n1EMT = SimNode<Real>::make("n1EMT", PhaseType::ABC, initialVoltage_n1);
	n1EMT->daeSetAbsoluteTolerance(voltage_absTolerance);

	// Node n2
    std::vector<Complex> initialVoltage_n2{ n3PF->voltage()(0,0)*PEAK1PH_TO_RMS3PH, 
											n3PF->voltage()(0,0)*SHIFT_TO_PHASE_B*PEAK1PH_TO_RMS3PH,
											n3PF->voltage()(0,0)*SHIFT_TO_PHASE_C*PEAK1PH_TO_RMS3PH
										  };
	auto n2EMT = SimNode<Real>::make("n2EMT", PhaseType::ABC, initialVoltage_n2);
	n2EMT->daeSetAbsoluteTolerance(voltage_absTolerance);

	// Slack
	auto slackEMT = EMT::Ph3::NetworkInjection::make("slackEMT", Logger::Level::info);
	MatrixComp voltageRef_slackEMT = MatrixComp::Zero(3, 1);
	voltageRef_slackEMT(0, 0) = Complex(Vnom, 0);
	voltageRef_slackEMT(1, 0) = Complex(Vnom, 0)*SHIFT_TO_PHASE_B;
	voltageRef_slackEMT(2, 0) = Complex(Vnom, 0)*SHIFT_TO_PHASE_C;
	slackEMT->setParameters(voltageRef_slackEMT, frequency);
	slackEMT->daeSetAbsoluteTolerance(current_absTolerance);

	// PiLine
	auto PiLineEMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::info);
	PiLineEMT->setParameters(line_rparam, line_lparam, line_cparam, line_gparam);
	PiLineEMT->daeSetAbsoluteTolerance(current_absTolerance);

	// RXLoad
	auto rxLoadEMT = EMT::Ph3::RXLoad::make("rxLoadEMT", p_load, q_load, Vnom, Logger::Level::info);
	rxLoadEMT->daeSetAbsoluteTolerance(current_absTolerance);

	// Topology
	slackEMT->connect({ n1EMT });
    PiLineEMT->connect({ n2EMT, n1EMT });
	rxLoadEMT->connect({ n2EMT });

	auto systemEMT = SystemTopology(frequency, SystemNodeList{n1EMT, n2EMT}, 
		SystemComponentList{slackEMT, PiLineEMT, rxLoadEMT});

	// Logger
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("v1", n1EMT->attribute("v"));
	//loggerEMT->logAttribute("dv1", n1EMT->attribute("dv"));
	loggerEMT->logAttribute("v2", n2EMT->attribute("v"));
	//loggerEMT->logAttribute("dv2", n2EMT->attribute("dv"));
	loggerEMT->logAttribute("v_line",  PiLineEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_slack", slackEMT->attribute("i_intf"));
	//loggerEMT->logAttribute("di_slack", slackEMT->attribute("di_intf"));

	// Set initial derivative of slack current
	slackEMT->setInitialComplexIntfCurrent(vsPF->intfCurrent()(0,0));

	Simulation simEMT(simNameEMT, Logger::Level::info);
	simEMT.setSystem(systemEMT);
	simEMT.setDomain(Domain::EMT);
	simEMT.setSolverType(Solver::Type::DAE);
	simEMT.setTimeStep(timeStepEMT);
	simEMT.setFinalTime(finalTimeEMT);
	simEMT.setRelativeTolerance(1e-10);
	simEMT.doSplitSubnets(false);
	simEMT.addLogger(loggerEMT);
	simEMT.run();


	return 0;
}

