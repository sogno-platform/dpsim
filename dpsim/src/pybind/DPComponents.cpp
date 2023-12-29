/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/pybind/DPComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim-models/IdentifiedObject.h>
#include <DPsim.h>
#include <dpsim-models/CSVReader.h>
#include <dpsim/pybind/Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addDPComponents(py::module_ mDP) {
	py::class_<CPS::DP::SimNode, std::shared_ptr<CPS::DP::SimNode>, CPS::TopologicalNode>(mDP, "SimNode", py::module_local())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::PhaseType>())
		.def(py::init<std::string, CPS::PhaseType, const std::vector<CPS::Complex>>())
		.def("set_initial_voltage", py::overload_cast<CPS::MatrixComp>(&CPS::DP::SimNode::setInitialVoltage, py::const_))
		.def("set_initial_voltage", py::overload_cast<CPS::Complex>(&CPS::DP::SimNode::setInitialVoltage, py::const_))
		.def("set_initial_voltage", py::overload_cast<CPS::Complex, int>(&CPS::DP::SimNode::setInitialVoltage, py::const_))
		.def("single_voltage", &CPS::DP::SimNode::singleVoltage, "phase_type"_a=CPS::PhaseType::Single)
		.def_readonly_static("gnd", &CPS::DP::SimNode::GND);


	py::module mDPPh1 = mDP.def_submodule("ph1", "single phase dynamic phasor models");
	addDPPh1Components(mDPPh1);

	py::module mDPPh3 = mDP.def_submodule("ph3", "triple phase dynamic phasor models");
	addDPPh3Components(mDPPh3);
}

void addDPPh1Components(py::module_ mDPPh1) {
	py::class_<CPS::DP::Ph1::VoltageSource, std::shared_ptr<CPS::DP::Ph1::VoltageSource>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "VoltageSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", py::overload_cast<CPS::Complex, CPS::Real>(&CPS::DP::Ph1::VoltageSource::setParameters), "V_ref"_a, "f_src"_a=0)
		.def("connect", &CPS::DP::Ph1::VoltageSource::connect)
		.def_property("V_ref", createAttributeGetter<CPS::Complex>("V_ref"), createAttributeSetter<CPS::Complex>("V_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

	py::class_<CPS::DP::Ph1::VoltageSourceNorton, std::shared_ptr<CPS::DP::Ph1::VoltageSourceNorton>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "VoltageSourceNorton", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", py::overload_cast<CPS::Complex, CPS::Real>(&CPS::DP::Ph1::VoltageSourceNorton::setParameters), "V_ref"_a, "f_src"_a=-1)
		.def("connect", &CPS::DP::Ph1::VoltageSourceNorton::connect);

	py::class_<CPS::DP::Ph1::CurrentSource, std::shared_ptr<CPS::DP::Ph1::CurrentSource>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "CurrentSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph1::CurrentSource::setParameters, "I_ref"_a)
		.def("connect", &CPS::DP::Ph1::CurrentSource::connect)
		.def_property("I_ref", createAttributeGetter<CPS::Complex>("I_ref"), createAttributeSetter<CPS::Complex>("I_ref"));

	py::class_<CPS::DP::Ph1::Resistor, std::shared_ptr<CPS::DP::Ph1::Resistor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Resistor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph1::Resistor::setParameters, "R"_a)
		.def("connect", &CPS::DP::Ph1::Resistor::connect)
		.def_property("R", createAttributeGetter<CPS::Real>("R"), createAttributeSetter<CPS::Real>("R"));

	py::class_<CPS::DP::Ph1::Capacitor, std::shared_ptr<CPS::DP::Ph1::Capacitor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Capacitor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph1::Capacitor::setParameters, "C"_a)
		.def("connect", &CPS::DP::Ph1::Capacitor::connect)
		.def_property("C", createAttributeGetter<CPS::Real>("C"), createAttributeSetter<CPS::Real>("C"));

	py::class_<CPS::DP::Ph1::Inductor, std::shared_ptr<CPS::DP::Ph1::Inductor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Inductor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph1::Inductor::setParameters, "L"_a)
		.def("connect", &CPS::DP::Ph1::Inductor::connect)
		.def_property("L", createAttributeGetter<CPS::Real>("L"), createAttributeSetter<CPS::Real>("L"));

	py::class_<CPS::DP::Ph1::ResIndSeries, std::shared_ptr<CPS::DP::Ph1::ResIndSeries>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "ResInductor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph1::ResIndSeries::setParameters, "R"_a, "L"_a)
		.def("connect", &CPS::DP::Ph1::ResIndSeries::connect);

	py::class_<CPS::DP::Ph1::NetworkInjection, std::shared_ptr<CPS::DP::Ph1::NetworkInjection>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "NetworkInjection", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_parameters", py::overload_cast<CPS::Complex, CPS::Real>(&CPS::DP::Ph1::NetworkInjection::setParameters), "V_ref"_a, "f_src"_a = 0)
		.def("connect", &CPS::DP::Ph1::NetworkInjection::connect);

	py::class_<CPS::DP::Ph1::PiLine, std::shared_ptr<CPS::DP::Ph1::PiLine>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "PiLine", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::DP::Ph1::PiLine::setParameters, "series_resistance"_a, "series_inductance"_a, "parallel_capacitance"_a=0, "parallel_conductance"_a=0)
		.def("connect", &CPS::DP::Ph1::PiLine::connect);

	py::class_<CPS::DP::Ph1::RXLoad, std::shared_ptr<CPS::DP::Ph1::RXLoad>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "RXLoad", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_parameters", py::overload_cast<CPS::Real, CPS::Real>(&CPS::DP::Ph1::RXLoad::setParameters), "active_power"_a, "reactive_power"_a)
        .def("set_parameters", py::overload_cast<CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::RXLoad::setParameters), "active_power"_a, "reactive_power"_a, "nominal_voltage"_a)
		.def("connect", &CPS::DP::Ph1::RXLoad::connect);

	py::class_<CPS::DP::Ph1::Switch, std::shared_ptr<CPS::DP::Ph1::Switch>, CPS::SimPowerComp<CPS::Complex>, CPS::Base::Ph1::Switch>(mDPPh1, "Switch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::DP::Ph1::Switch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("open", &CPS::DP::Ph1::Switch::open)
		.def("close", &CPS::DP::Ph1::Switch::close)
		.def("connect", &CPS::DP::Ph1::Switch::connect);

	py::class_<CPS::DP::Ph1::varResSwitch, std::shared_ptr<CPS::DP::Ph1::varResSwitch>, CPS::SimPowerComp<CPS::Complex>, CPS::Base::Ph1::Switch>(mDPPh1, "varResSwitch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::DP::Ph1::varResSwitch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("open", &CPS::DP::Ph1::varResSwitch::open)
		.def("close", &CPS::DP::Ph1::varResSwitch::close)
		.def("set_init_parameters", &CPS::DP::Ph1::varResSwitch::setInitParameters, "time_step"_a)
		.def("connect", &CPS::DP::Ph1::varResSwitch::connect);

	py::class_<CPS::DP::Ph1::SynchronGeneratorTrStab, std::shared_ptr<CPS::DP::Ph1::SynchronGeneratorTrStab>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "SynchronGeneratorTrStab", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_standard_parameters_PU", &CPS::DP::Ph1::SynchronGeneratorTrStab::setStandardParametersPU,
				"nom_power"_a, "nom_volt"_a, "nom_freq"_a, "Xpd"_a, "inertia"_a, "Rs"_a=0, "D"_a=0)
		.def("set_fundamental_parameters_PU", &CPS::DP::Ph1::SynchronGeneratorTrStab::setFundamentalParametersPU,
				"nom_power"_a, "nom_volt"_a, "nom_freq"_a, "Ll"_a, "Lmd"_a, "Llfd"_a, "H"_a, "D"_a = 0)
		.def("set_initial_values", &CPS::DP::Ph1::SynchronGeneratorTrStab::setInitialValues, "elec_power"_a, "mech_power"_a)
		.def("connect", &CPS::DP::Ph1::SynchronGeneratorTrStab::connect)
		.def("set_model_flags", &CPS::DP::Ph1::SynchronGeneratorTrStab::setModelFlags, "convert_with_omega_mech"_a)
		.def("set_reference_omega", [](CPS::DP::Ph1::SynchronGeneratorTrStab &gen, std::string refOmegaName, CPS::IdentifiedObject::Ptr refOmegaComp,
			std::string refDeltaName, CPS::IdentifiedObject::Ptr refDeltaComp) {
				gen.setReferenceOmega(refOmegaComp->attributeTyped<CPS::Real>(refOmegaName), refDeltaComp->attributeTyped<CPS::Real>(refDeltaName));
			}, "ref_omega_name"_a="w_r", "ref_omage_comp"_a, "ref_delta_name"_a="delta_r", "ref_delta_comp"_a);

	py::class_<CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR, std::shared_ptr<CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>>(mDPPh1, "ReducedOrderSynchronGeneratorVBR", py::multiple_inheritance());

	py::class_<CPS::DP::Ph1::SynchronGenerator3OrderVBR, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator3OrderVBR>, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>(mDPPh1, "SynchronGenerator3OrderVBR", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator3OrderVBR::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Td0_t"_a);
		
	py::class_<CPS::DP::Ph1::SynchronGenerator4OrderVBR, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator4OrderVBR>, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>(mDPPh1, "SynchronGenerator4OrderVBR", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator4OrderVBR::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a);

	py::class_<CPS::DP::Ph1::SynchronGenerator5OrderVBR, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator5OrderVBR>, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>(mDPPh1, "SynchronGenerator5OrderVBR", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator5OrderVBR::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a, "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a);

	py::class_<CPS::DP::Ph1::SynchronGenerator6aOrderVBR, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator6aOrderVBR>, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>(mDPPh1, "SynchronGenerator6aOrderVBR", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator6aOrderVBR::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a, "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a);

	py::class_<CPS::DP::Ph1::SynchronGenerator6bOrderVBR, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator6bOrderVBR>, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>(mDPPh1, "SynchronGenerator6bOrderVBR", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator6bOrderVBR::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a, "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a=0);

	py::class_<CPS::DP::Ph1::SynchronGenerator4OrderTPM, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator4OrderTPM>, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>, CPS::MNASyncGenInterface>(mDPPh1, "SynchronGenerator4OrderTPM", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator4OrderTPM::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a);

	py::class_<CPS::DP::Ph1::SynchronGenerator4OrderPCM, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator4OrderPCM>, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>, CPS::MNASyncGenInterface>(mDPPh1, "SynchronGenerator4OrderPCM", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator4OrderPCM::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a)
		.def("connect", &CPS::DP::Ph1::SynchronGenerator4OrderPCM::connect);

	py::class_<CPS::DP::Ph1::SynchronGenerator6OrderPCM, std::shared_ptr<CPS::DP::Ph1::SynchronGenerator6OrderPCM>, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>, CPS::MNASyncGenInterface>(mDPPh1, "SynchronGenerator6OrderPCM", py::multiple_inheritance())
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_operational_parameters_per_unit", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::SynchronGenerator6OrderPCM::setOperationalParametersPerUnit), "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a, "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a, "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a=0)
		.def("connect", &CPS::DP::Ph1::SynchronGenerator6OrderPCM::connect);

	py::class_<CPS::DP::Ph1::AvVoltageSourceInverterDQ, std::shared_ptr<CPS::DP::Ph1::AvVoltageSourceInverterDQ>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "AvVoltageSourceInverterDQ", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool>(), "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off, "with_trafo"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("set_parameters", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::setParameters, "sys_omega"_a, "sys_volt_nom"_a, "p_ref"_a, "q_ref"_a)
		.def("set_filter_parameters", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::setFilterParameters, "Lf"_a, "Cf"_a, "Rf"_a, "Rc"_a)
		.def("set_controller_parameters", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::setControllerParameters,
			"Kp_pll"_a, "Ki_pll"_a, "Kp_power_ctrl"_a, "Ki_power_ctrl"_a, "Kp_curr_ctrl"_a, "Ki_curr_ctrl"_a, "omega_cutoff"_a)
		.def("set_transformer_parameters", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::setTransformerParameters,
			"nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "rated_power"_a, "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a)
		.def("set_initial_state_values", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::setInitialStateValues,
			"p_init"_a, "q_init"_a, "phi_d_init"_a, "phi_q_init"_a, "gamma_d_init"_a, "gamma_q_init"_a)
		.def("with_control", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::withControl)
		.def("connect", &CPS::DP::Ph1::AvVoltageSourceInverterDQ::connect);

	py::class_<CPS::DP::Ph1::VSIVoltageControlDQ, std::shared_ptr<CPS::DP::Ph1::VSIVoltageControlDQ>, CPS::SimPowerComp<CPS::Complex>, CPS::Base::VSIVoltageSourceInverterDQ<CPS::Complex>>(mDPPh1, "VSIVoltageControlDQ", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool, CPS::Bool>(), "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off, "model_as_current_source"_a=false, "with_interface_resistor"_a=false) // cppcheck-suppress assignBoolToPointer
		.def("connect", &CPS::DP::Ph1::VSIVoltageControlDQ::connect);

	py::class_<CPS::DP::Ph1::Inverter, std::shared_ptr<CPS::DP::Ph1::Inverter>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Inverter", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_parameters", &CPS::DP::Ph1::Inverter::setParameters, "carrier_harms"_a, "modul_harms"_a, "input_voltage"_a, "ratio"_a, "phase"_a)
		.def("connect", &CPS::DP::Ph1::Inverter::connect);

	py::class_<CPS::DP::Ph1::Transformer, std::shared_ptr<CPS::DP::Ph1::Transformer>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "Transformer", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def(py::init<std::string, std::string, CPS::Logger::Level>(), "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off) // cppcheck-suppress assignBoolToPointer
		.def("set_parameters", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::DP::Ph1::Transformer::setParameters), "nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a)
		.def("connect", &CPS::DP::Ph1::Transformer::connect);
}

void addDPPh3Components(py::module_ mDPPh3) {

	#ifdef WITH_SUNDIALS

	py::class_<CPS::DP::Ph3::SynchronGeneratorDQODE, std::shared_ptr<CPS::DP::Ph3::SynchronGeneratorDQODE>, CPS::SimPowerComp<CPS::Complex>>(mDPPh3, "SynchronGeneratorDQODE", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_parameters_fundamental_per_unit", &CPS::DP::Ph3::SynchronGeneratorDQODE::setParametersFundamentalPerUnit,
				"nom_power"_a, "nom_volt"_a, "nom_freq"_a, "pole_number"_a, "nom_field_cur"_a,
				"Rs"_a, "Ll"_a, "Lmd"_a, "Lmq"_a, "Rfd"_a, "Llfd"_a, "Rkd"_a, "Llkd"_a,
				"Rkq1"_a, "Llkq1"_a, "Rkq2"_a, "Llkq2"_a, "inertia"_a, "init_active_power"_a,
				"init_reactive_power"_a, "init_terminal_volt"_a, "init_volt_angle"_a, "init_mech_power"_a)
		.def("connect", &CPS::DP::Ph3::SynchronGeneratorDQODE::connect);

	#endif

	py::class_<CPS::DP::Ph3::SynchronGeneratorDQTrapez, std::shared_ptr<CPS::DP::Ph3::SynchronGeneratorDQTrapez>, CPS::SimPowerComp<CPS::Complex>>(mDPPh3, "SynchronGeneratorDQTrapez", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_parameters_fundamental_per_unit", &CPS::DP::Ph3::SynchronGeneratorDQTrapez::setParametersFundamentalPerUnit,
				"nom_power"_a, "nom_volt"_a, "nom_freq"_a, "pole_number"_a, "nom_field_cur"_a,
				"Rs"_a, "Ll"_a, "Lmd"_a, "Lmq"_a, "Rfd"_a, "Llfd"_a, "Rkd"_a, "Llkd"_a,
				"Rkq1"_a, "Llkq1"_a, "Rkq2"_a, "Llkq2"_a, "inertia"_a, "init_active_power"_a,
				"init_reactive_power"_a, "init_terminal_volt"_a, "init_volt_angle"_a, "init_mech_power"_a)
		.def("connect", &CPS::DP::Ph3::SynchronGeneratorDQTrapez::connect);

	py::class_<CPS::DP::Ph3::SeriesResistor, std::shared_ptr<CPS::DP::Ph3::SeriesResistor>, CPS::SimPowerComp<CPS::Complex>>(mDPPh3, "SeriesResistor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph3::SeriesResistor::setParameters, "R"_a)
		.def("connect", &CPS::DP::Ph3::SeriesResistor::connect);

	py::class_<CPS::DP::Ph3::SeriesSwitch, std::shared_ptr<CPS::DP::Ph3::SeriesSwitch>, CPS::SimPowerComp<CPS::Complex>, CPS::Base::Ph1::Switch>(mDPPh3, "SeriesSwitch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::DP::Ph3::SeriesSwitch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("open", &CPS::DP::Ph3::SeriesSwitch::open)
		.def("close", &CPS::DP::Ph3::SeriesSwitch::close)
		.def("connect", &CPS::DP::Ph3::SeriesSwitch::connect);
}
