/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP;

void DP_Ph1_VSI2_4bus_SampleGrid() {
	// Define simulation scenario
	Real timeStep = 0.0001;
	Real finalTime = 1;
	String simName = "DP_Ph1_VSI2_4BUS_SampleGrid-3kv";
	Logger::setLogDir("logs/" + simName);

	Real Vnom = 3300.;

	// Nodes
	auto n1 = SimNode::make("n1");
	auto n2 = SimNode::make("n2");
	auto n3 = SimNode::make("n3");
	auto n4 = SimNode::make("n4");
	auto n5 = SimNode::make("n5");
	auto n6 = SimNode::make("n6");
	auto n7 = SimNode::make("n7");

	// Components
	auto vsi = Ph1::AvVoltageSourceInverterDQ::make("vsi", Logger::Level::debug);
	Real Pref1 = 30000;
	Real Qref1 = 200;
	Real Pref2 = 40000;
	Real Qref2 = 1000;

	auto vs = Ph1::VoltageSource::make("vs");
	vs->setParameters(Complex(Vnom, 0));
	auto vsi2 = Ph1::AvVoltageSourceInverterDQ::make("vsi2", Logger::Level::debug);

	Real rf_param = 0.01;
	Real lf_param = 0.928e-3;
	Real cf_param = 789.3e-6;
	Real rc_param = 0.5;

	auto piline = Ph1::PiLine::make("piline");
	Real rline_param = 0.04;
	piline->setParameters(rline_param,lf_param*0.1,cf_param);

	auto rline2 = Ph1::Resistor::make("rline2");
	rline2->setParameters(rline_param);

	auto rline3 = Ph1::Resistor::make("rline3");
	rline3->setParameters(rline_param);

	auto rline4 = Ph1::Resistor::make("rline4");
	rline4->setParameters(rline_param);


	auto rload = Ph1::Resistor::make("rload");
	Real rload_param_1kw_220v = 47.6568;
	rload->setParameters(rload_param_1kw_220v);

	auto rload2 = Ph1::Resistor::make("rload2");
	rload2->setParameters(rload_param_1kw_220v);

	auto Lload = Ph1::Inductor::make("Lload");
	Real Lload_param_100var_220v = 0.01516;
	Lload->setParameters(Lload_param_100var_220v);

	auto Lload2 = Ph1::Inductor::make("Lload2");
	Lload2->setParameters(Lload_param_100var_220v);

	vsi->setParameters(2 * M_PI * 50, Vnom, Pref1, Qref1);
	vsi->setControllerParameters(0.25, 2, 0.001, 0.08, 3.77, 1400, 2 * M_PI * 50);
	vsi->setFilterParameters(lf_param, cf_param, rf_param, rc_param);

	vsi2->setParameters(2 * M_PI * 50, Vnom, Pref2, Qref2);
	vsi2->setControllerParameters(0.25, 2, 0.001, 0.08, 3.77, 1400, 2 * M_PI * 50);
	vsi2->setFilterParameters(lf_param, cf_param, rf_param, rc_param);

	// 4Bus case study
	vs->connect(SimNode::List{ SimNode::GND, n5 });

	vsi->connect(SimNode::List{ n1 });
	piline->connect(SimNode::List{ n5, n1 });

	rline2->connect(SimNode::List{ n1, n2 });

	rload->connect(SimNode::List{ n2, n6 });
	Lload->connect(SimNode::List{ n6, SimNode::GND });

	rline3->connect(SimNode::List{ n2, n3 });

	vsi2->connect(SimNode::List{ n3 });

	rline4->connect(SimNode::List{ n3, n4 });
	rload2->connect(SimNode::List{ n4, n7 });
	Lload2->connect(SimNode::List{ n7, SimNode::GND });


	auto sys = SystemTopology(50, SystemNodeList{ n1, n2, n3, n4, n5, n6, n7 },
		SystemComponentList{ vsi, vs, piline, rline2, rline3, rline4, rload, rload2,
							Lload, Lload2, vsi2 });
	for (auto node : sys.mNodes) {
		node->setInitialVoltage(Complex(Vnom, 0));
	}
	// Logging
	auto logger = CPS::DataLogger::make(simName);
	// currents
	/*logger->logAttribute("i_vs", vs->attribute("i_intf"));
	logger->logAttribute("i_vsi", vsi->attribute("i_intf"));
	logger->logAttribute("i_vsi2", vsi2->attribute("i_intf"));
	logger->logAttribute("iload", rload->attribute("i_intf"));
	logger->logAttribute("iqload", Lload->attribute("i_intf"));*/
	// nodal voltages
	logger->logAttribute("vBus1", n1->attribute("v"));
	logger->logAttribute("v_slack", n5->attribute("v"));
	logger->logAttribute("vBus2", n2->attribute("v"));
	logger->logAttribute("vBus3", n3->attribute("v"));
	logger->logAttribute("vBus4", n4->attribute("v"));
	// power outputs
	logger->logAttribute("P", vsi->attribute("p"));
	logger->logAttribute("Q", vsi->attribute("q"));
	logger->logAttribute("P2", vsi2->attribute("p"));
	logger->logAttribute("Q2", vsi2->attribute("q"));
	logger->logAttribute("P_ref", vsi->attribute("P_ref"));
	logger->logAttribute("Q_ref", vsi->attribute("Q_ref"));
	logger->logAttribute("P_ref2", vsi2->attribute("P_ref"));
	logger->logAttribute("Q_ref2", vsi2->attribute("Q_ref"));
	// states of controller
	logger->logAttribute("theta", vsi->attribute("theta"));
	logger->logAttribute("phi_pll", vsi->attribute("phipll"));
	logger->logAttribute("phid", vsi->attribute("phid"));
	logger->logAttribute("phiq", vsi->attribute("phiq"));
	logger->logAttribute("gammad", vsi->attribute("gammad"));
	logger->logAttribute("gammaq", vsi->attribute("gammaq"));
	logger->logAttribute("ifdq", vsi->attribute("ifdq"));
	logger->logAttribute("igdq", vsi->attribute("igdq"));
	logger->logAttribute("Vsdq", vsi->attribute("Vsdq"));


	// frequency
	logger->logAttribute("freq_vsi1", vsi->attribute("freq"));
	logger->logAttribute("freq_vsi2", vsi2->attribute("freq"));
	// output voltages
	logger->logAttribute("vsi", vsi->attribute("v_intf"));
	logger->logAttribute("vsi2", vsi2->attribute("v_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	DP_Ph1_VSI2_4bus_SampleGrid();
}
