/**
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;
using namespace CPS::EMT::Ph3;

void EMT_Ph3_VSI2_4bus_SampleGrid() {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 0.6;
	String simName = "EMT_Ph3_VSI2_4BUS_SampleGrid";
	Logger::setLogDir("logs/" + simName);
	Real Vnom = 20e3;
	Real Vnom_pv = 2e3;
	// Nodes
	std::vector<Complex> initialVoltage{ Complex(Vnom, 0), Complex(Vnom, 0), Complex(Vnom, 0) };
	std::vector<Complex> initialVoltage_DG{ Complex(Vnom_pv, 0), Complex(Vnom_pv, 0), Complex(Vnom_pv, 0) };

	auto n1 = Node::make("n1", PhaseType::ABC, initialVoltage);
	auto n2 = Node::make("n2", PhaseType::ABC, initialVoltage);
	auto n3 = Node::make("n3", PhaseType::ABC, initialVoltage);
	auto n4 = Node::make("n4", PhaseType::ABC, initialVoltage);
	auto n5 = Node::make("n5", PhaseType::ABC, initialVoltage);
	auto n6 = Node::make("n6", PhaseType::ABC, initialVoltage);
	auto n7 = Node::make("n7", PhaseType::ABC, initialVoltage);
	auto n_trans1 = Node::make("n_trans1", PhaseType::ABC, initialVoltage_DG);
	auto n_trans2 = Node::make("n_trans2", PhaseType::ABC, initialVoltage_DG);

	// Components
	auto vsi = AvVoltageSourceInverterDQ::make("vsi", Logger::Level::info);
	Real Pref1 = 300000;
	Real Qref1 = 200;
	Real Pref2 = 400000;
	Real Qref2 = 1000;

	auto vs = VoltageSource::make("vs", Logger::Level::info);
	vs->setParameters(Complex(Vnom, 0), 50);
	auto vsi2 = AvVoltageSourceInverterDQ::make("vsi2", Logger::Level::debug);
	auto trans_DG1 = Ph3::Transformer::make("trans_pv1", Logger::Level::debug);
	auto trans_DG2 = Ph3::Transformer::make("trans_pv2", Logger::Level::debug);

	Matrix rf_param = Matrix::Zero(3, 3);
	rf_param <<
		0.01, 0, 0,
		0, 0.01, 0,
		0, 0, 0.01;

	Matrix lf_param = Matrix(3, 3);
	lf_param <<
		0.928e-3, 0, 0,
		0, 0.928e-3, 0,
		0, 0, 0.928e-3;

	Matrix cf_param = Matrix(3, 3);
	cf_param <<
		789.3e-6, 0, 0,
		0, 789.3e-6, 0,
		0, 0, 789.3e-6;

	Matrix rc_param = Matrix::Zero(3, 3);
	rc_param <<
		0.5, 0, 0,
		0, 0.5, 0,
		0, 0, 0.5;

	trans_DG1->setParameters(Vnom_pv / Vnom, 0., rc_param, lf_param*0.01);
	trans_DG2->setParameters(Vnom_pv / Vnom, 0., rc_param, lf_param * 0.01);

	//trans_inductor->setParameters(lf_param*0.01);
	auto piline = PiLine::make("piline", Logger::Level::debug);
	Matrix rline_param = Matrix::Zero(3, 3);
	rline_param <<
		0.04, 0, 0,
		0, 0.04, 0,
		0, 0, 0.04;

	piline->setParameters(rline_param*10, lf_param*0.0001, cf_param);

	auto rline2 = Resistor::make("rline2", Logger::Level::info);
	rline2->setParameters(rline_param);

	auto rline3 = Resistor::make("rline3", Logger::Level::info);
	rline3->setParameters(rline_param);

	auto rline4 = Resistor::make("rline4", Logger::Level::info);
	rline4->setParameters(rline_param);


	auto rload = Resistor::make("rload", Logger::Level::info);
	Matrix rload_param_1kw_220v = Matrix::Zero(3, 3);

	rload_param_1kw_220v <<
		47.6568, 0, 0,
		0, 47.6568, 0,
		0, 0, 47.6568;

	rload->setParameters(rload_param_1kw_220v);

	auto rload2 = Resistor::make("rload2", Logger::Level::info);
	rload2->setParameters(rload_param_1kw_220v);

	auto Lload = Inductor::make("Lload", Logger::Level::info);
	Matrix Lload_param_100var_220v = Matrix(3, 3);
	Lload_param_100var_220v <<
		0.01516, 0, 0,
		0, 0.01516, 0,
		0, 0, 0.01516;

	Lload->setParameters(Lload_param_100var_220v);

	auto Lload2 = Inductor::make("Lload2", Logger::Level::info);
	Lload2->setParameters(Lload_param_100var_220v);

	vsi->setParameters(2 * M_PI * 50, Complex(Vnom_pv, 0),
		Pref1, Qref1, 0.25, 2, 0.001, 0.08, 3.77, 1400,
		lf_param, cf_param, rf_param, rc_param);

	vsi2->setParameters(2 * M_PI * 50, Complex(Vnom, 0),
		Pref2, Qref2, 0.25, 2, 0.001, 0.08, 3.77, 1400,
		lf_param, cf_param, rf_param, rc_param);

	Matrix p_load = Matrix::Zero(3, 3);
	p_load <<
		1e6, 0, 0,
		0, 1e6, 0,
		0, 0, 1e6;
	Matrix q_load = Matrix::Zero(3, 3);
	q_load <<
		1e2, 0, 0,
		0, 1e2, 0,
		0, 0, 1e2;

	auto rxLoad = RXLoad::make("rxload", p_load, q_load, 20e3, Logger::Level::info);

	// 4Bus case study
	vsi->connect(Node::List{ n_trans1 });
	trans_DG1->connect(Node::List{ n_trans1, n1 });
	//trans_inductor->connect(Node::List{ n_trans1, n1 });
	rxLoad->connect(Node::List{ n1 });
	vs->connect(Node::List{ Node::GND, n2 });
	piline->connect(Node::List{ n1, n2 });

	rline2->connect(Node::List{ n1, n3 });

	rload->connect(Node::List{ n3, n6 });
	Lload->connect(Node::List{ n6, Node::GND });

	rline3->connect(Node::List{ n3, n4 });

	vsi2->connect(Node::List{ n_trans2 });
	trans_DG2->connect(Node::List{ n_trans2, n4 });

	rline4->connect(Node::List{ n3, n5 });
	rload2->connect(Node::List{ n5, n7 });
	Lload2->connect(Node::List{ n7, Node::GND });


	// Define system topology
	auto sys = SystemTopology(50, SystemNodeList{ n1, n2, n3, n4, n5, n6, n7,n_trans1, n_trans2 },
		SystemComponentList{ vsi, vs, piline, rline2, rline3, rline4, rload, rload2,
							Lload, Lload2, vsi2, trans_DG1,trans_DG2, rxLoad });

	// extract initial voltages to EMT system
	//NRpolarSolver nrpSolver(simName + ".pf", sysPF, 1, Domain::Static, Logger::Level::off);
	//nrpSolver.writePFResultsToDynamicTopology(sysPF, sys);

	// Logging
	//auto logger = DataLogger::make(simName);
	auto logger_vsi = DataLogger::make(simName+"_pv1");
	auto logger_vsi2 = DataLogger::make(simName + "_pv2");
	//// currents
	//logger->addAttribute("i_vs", vs->attribute("i_intf"));
	//logger->addAttribute("i_vsi", vsi->attribute("i_intf"));
	//logger->addAttribute("i_vsi2", vsi2->attribute("i_intf"));
	//logger->addAttribute("iload", rload->attribute("i_intf"));
	//logger->addAttribute("iqload", Lload->attribute("i_intf"));
	//// nodal voltages
	//logger->addAttribute("v_slack", n2->attribute("v"));
	//logger->addAttribute("vBus1", n1->attribute("v"));
	//logger->addAttribute("vBus2", n3->attribute("v"));
	//logger->addAttribute("vBus3", n5->attribute("v"));
	//// power outputs
	logger_vsi->addAttribute("P", vsi->attribute("p"));
	logger_vsi->addAttribute("Q", vsi->attribute("q"));
	logger_vsi->addAttribute("P_ref", vsi->attribute("P_ref"));
	logger_vsi->addAttribute("Q_ref", vsi->attribute("Q_ref"));
	//// vsi
	logger_vsi->addAttribute("Vcabc", vsi->attribute("Vcabc"));
	logger_vsi->addAttribute("Vcdq", vsi->attribute("Vcdq"));
	logger_vsi->addAttribute("Vsdq", vsi->attribute("Vsdq"));
	logger_vsi->addAttribute("ifdq", vsi->attribute("ifdq"));
	logger_vsi->addAttribute("igdq", vsi->attribute("igdq"));

	logger_vsi2->addAttribute("P", vsi2->attribute("p"));
	logger_vsi->addAttribute("Q", vsi2->attribute("q"));
	logger_vsi2->addAttribute("P_ref", vsi2->attribute("P_ref"));
	logger_vsi->addAttribute("Q_ref", vsi2->attribute("Q_ref"));
	//// vsi
	logger_vsi2->addAttribute("Vcabc", vsi2->attribute("Vcabc"));
	logger_vsi2->addAttribute("Vcdq", vsi2->attribute("Vcdq"));
	logger_vsi2->addAttribute("Vsdq", vsi2->attribute("Vsdq"));
	logger_vsi2->addAttribute("ifdq", vsi2->attribute("ifdq"));
	logger_vsi2->addAttribute("igdq", vsi2->attribute("igdq"));

	//// states of controller
	//logger->addAttribute("theta", vsi->attribute("theta"));
	//logger->addAttribute("phi_pll", vsi->attribute("phipll"));
	//logger->addAttribute("phid", vsi->attribute("phid"));
	//logger->addAttribute("phiq", vsi->attribute("phiq"));
	//logger->addAttribute("gammad", vsi->attribute("gammad"));
	//logger->addAttribute("gammaq", vsi->attribute("gammaq"));
	//// frequency
	logger_vsi->addAttribute("freq_vsi1", vsi->attribute("freq"));
	//logger->addAttribute("freq_vsi2", vsi2->attribute("freq"));
	//// output voltages
	//logger->addAttribute("vsi", vsi->attribute("v_intf"));
	//logger->addAttribute("vsi_dq", vsi->attribute("Vsdq"));
	//logger->addAttribute("vsi2", vsi2->attribute("v_intf"));
	//logger->addAttribute("vsi2_dq", vsi2->attribute("Vsdq"));

	Simulation sim(simName, sys, timeStep, finalTime, Domain::EMT);
	//sim.addLogger(logger);
	sim.addLogger(logger_vsi);
	sim.addLogger(logger_vsi2);
	sim.run();
}

int main(int argc, char* argv[]) {
	EMT_Ph3_VSI2_4bus_SampleGrid();
}