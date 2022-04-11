/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <SPComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <cps/CSVReader.h>
#include <Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addSPComponents(py::module_ mSP) {
	py::module mSPPh1 = mSP.def_submodule("ph1", "single phase static phasor models");
	addSPPh1Components(mSPPh1);
	py::module mSPPh3 = mSP.def_submodule("ph3", "three phase static phasor models");
	addSPPh3Components(mSPPh3);
}

void addSPPh1Components(py::module_ mSPPh1) {
	//SP Ph1 Components
	py::class_<CPS::SP::Ph1::VoltageSource, std::shared_ptr<CPS::SP::Ph1::VoltageSource>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "VoltageSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", py::overload_cast<CPS::Complex, CPS::Real>(&CPS::SP::Ph1::VoltageSource::setParameters), "V_ref"_a, "f_src"_a = 0)
		.def("connect", &CPS::SP::Ph1::VoltageSource::connect)
		.def_property("V_ref", createAttributeGetter<CPS::Complex>("V_ref"), createAttributeSetter<CPS::Complex>("V_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

	py::class_<CPS::SP::Ph1::Resistor, std::shared_ptr<CPS::SP::Ph1::Resistor>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Resistor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph1::Resistor::setParameters, "R"_a)
		.def("connect", &CPS::SP::Ph1::Resistor::connect)
		.def_property("R", createAttributeGetter<CPS::Real>("R"), createAttributeSetter<CPS::Complex>("R"));

	py::class_<CPS::SP::Ph1::Capacitor, std::shared_ptr<CPS::SP::Ph1::Capacitor>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Capacitor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph1::Capacitor::setParameters, "C"_a)
		.def("connect", &CPS::SP::Ph1::Capacitor::connect)
		.def_property("C", createAttributeGetter<CPS::Real>("C"), createAttributeSetter<CPS::Real>("C"));

	py::class_<CPS::SP::Ph1::Inductor, std::shared_ptr<CPS::SP::Ph1::Inductor>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Inductor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph1::Inductor::setParameters, "L"_a)
		.def("connect", &CPS::SP::Ph1::Inductor::connect)
		.def_property("L", createAttributeGetter<CPS::Real>("L"), createAttributeSetter<CPS::Real>("L"));

	py::class_<CPS::SP::Ph1::NetworkInjection, std::shared_ptr<CPS::SP::Ph1::NetworkInjection>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "NetworkInjection", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", py::overload_cast<CPS::Real>(&CPS::SP::Ph1::NetworkInjection::setParameters), "voltage_set_point"_a)
        .def("set_parameters", py::overload_cast<CPS::Complex, CPS::Real>(&CPS::SP::Ph1::NetworkInjection::setParameters), "V_ref"_a, "f_src"_a = 0)
		.def("set_base_voltage", &CPS::SP::Ph1::NetworkInjection::setBaseVoltage, "base_voltage"_a)
		.def("connect", &CPS::SP::Ph1::NetworkInjection::connect)
		.def("modify_power_flow_bus_type", &CPS::SP::Ph1::NetworkInjection::modifyPowerFlowBusType, "bus_type"_a);

	py::class_<CPS::SP::Ph1::PiLine, std::shared_ptr<CPS::SP::Ph1::PiLine>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "PiLine", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::PiLine::setParameters, "R"_a, "L"_a, "C"_a=-1, "G"_a=-1)
		.def("set_base_voltage", &CPS::SP::Ph1::PiLine::setBaseVoltage, "base_voltage"_a)
		.def("connect", &CPS::SP::Ph1::PiLine::connect);

	py::class_<CPS::SP::Ph1::Shunt, std::shared_ptr<CPS::SP::Ph1::Shunt>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Shunt", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::Shunt::setParameters, "G"_a, "B"_a)
		.def("set_base_voltage", &CPS::SP::Ph1::Shunt::setBaseVoltage, "base_voltage"_a)
		.def("connect", &CPS::SP::Ph1::Shunt::connect);

	py::class_<CPS::SP::Ph1::Load, std::shared_ptr<CPS::SP::Ph1::Load>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Load", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::Load::setParameters, "active_power"_a, "reactive_power"_a, "nominal_voltage"_a)
		.def("modify_power_flow_bus_type", &CPS::SP::Ph1::Load::modifyPowerFlowBusType, "bus_type"_a)
		.def("connect", &CPS::SP::Ph1::Load::connect);

	py::class_<CPS::SP::Ph1::Switch, std::shared_ptr<CPS::SP::Ph1::Switch>, CPS::SimPowerComp<CPS::Complex>, CPS::Base::Ph1::Switch>(mSPPh1, "Switch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::Switch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("open", &CPS::SP::Ph1::Switch::open)
		.def("close", &CPS::SP::Ph1::Switch::close)
		.def("connect", &CPS::SP::Ph1::Switch::connect);

	py::class_<CPS::SP::Ph1::SynchronGenerator, std::shared_ptr<CPS::SP::Ph1::SynchronGenerator>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "SynchronGenerator", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", (&CPS::SP::Ph1::SynchronGenerator::setParameters), "rated_apparent_power"_a, "rated_voltage"_a, "set_point_active_power"_a,
			"set_point_voltage"_a, "powerflow_bus_type"_a, "set_point_reactive_power"_a=0)
        .def("set_base_voltage", &CPS::SP::Ph1::SynchronGenerator::setBaseVoltage, "base_voltage"_a)
		.def("connect", &CPS::SP::Ph1::SynchronGenerator::connect)
		.def("modify_power_flow_bus_type", &CPS::SP::Ph1::SynchronGenerator::modifyPowerFlowBusType, "bus_type"_a)
		.def("get_apparent_power", &CPS::SP::Ph1::SynchronGenerator::getApparentPower);

	py::class_<CPS::SP::Ph1::varResSwitch, std::shared_ptr<CPS::SP::Ph1::varResSwitch>, CPS::SimPowerComp<CPS::Complex>, CPS::Base::Ph1::Switch>(mSPPh1, "varResSwitch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::varResSwitch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("open", &CPS::SP::Ph1::varResSwitch::open)
		.def("close", &CPS::SP::Ph1::varResSwitch::close)
		.def("set_init_parameters", &CPS::SP::Ph1::varResSwitch::setInitParameters, "time_step"_a)
		.def("connect", &CPS::SP::Ph1::varResSwitch::connect);

	py::class_<CPS::SP::Ph1::SynchronGeneratorTrStab, std::shared_ptr<CPS::SP::Ph1::SynchronGeneratorTrStab>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "SynchronGeneratorTrStab", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_standard_parameters_PU", &CPS::SP::Ph1::SynchronGeneratorTrStab::setStandardParametersPU,
				"nom_power"_a, "nom_volt"_a, "nom_freq"_a, "Xpd"_a, "inertia"_a, "Rs"_a=0, "D"_a=0)
		.def("set_initial_values", &CPS::SP::Ph1::SynchronGeneratorTrStab::setInitialValues, "elec_power"_a, "mech_power"_a)
		.def("connect", &CPS::SP::Ph1::SynchronGeneratorTrStab::connect)
		.def("set_model_flags", &CPS::SP::Ph1::SynchronGeneratorTrStab::setModelFlags, "use_omega_ref"_a, "convert_with_omega_mech"_a)
		.def("set_reference_omega", [](CPS::SP::Ph1::SynchronGeneratorTrStab &gen, std::string refOmegaName, CPS::IdentifiedObject::Ptr refOmegaComp,
			std::string refDeltaName, CPS::IdentifiedObject::Ptr refDeltaComp) {
				gen.setReferenceOmega(refOmegaComp->attribute<CPS::Real>(refOmegaName), refDeltaComp->attribute<CPS::Real>(refDeltaName));
			}, "ref_omega_name"_a="w_r", "ref_omage_comp"_a, "ref_delta_name"_a="delta_r", "ref_delta_comp"_a);

	py::class_<CPS::SP::Ph1::AvVoltageSourceInverterDQ, std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "AvVoltageSourceInverterDQ", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool>(), "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off, "with_trafo"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("set_parameters", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::setParameters, "sys_omega"_a, "sys_volt_nom"_a, "p_ref"_a, "q_ref"_a)
		.def("set_filter_parameters", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::setFilterParameters, "Lf"_a, "Cf"_a, "Rf"_a, "Rc"_a)
		.def("set_controller_parameters", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::setControllerParameters,
			"Kp_pll"_a, "Ki_pll"_a, "Kp_power_ctrl"_a, "Ki_power_ctrl"_a, "Kp_curr_ctrl"_a, "Ki_curr_ctrl"_a, "omega_cutoff"_a)
		.def("set_transformer_parameters", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::setTransformerParameters,
			"nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "rated_power"_a, "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a)
		.def("set_initial_state_values", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::setInitialStateValues,
			"p_init"_a, "q_init"_a, "phi_d_init"_a, "phi_q_init"_a, "gamma_d_init"_a, "gamma_q_init"_a)
		.def("with_control", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::withControl)
		.def("connect", &CPS::SP::Ph1::AvVoltageSourceInverterDQ::connect);

	py::class_<CPS::SP::Ph1::Transformer, std::shared_ptr<CPS::SP::Ph1::Transformer>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Transformer", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool>(), "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off, "with_resistive_losses"_a = false) // cppcheck-suppress assignBoolToPointer
		.def("set_parameters", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::SP::Ph1::Transformer::setParameters), "nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a)
		.def("set_parameters", py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real, CPS::Real>(&CPS::SP::Ph1::Transformer::setParameters), "nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "rated_power"_a, "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a)
		.def("set_base_voltage", &CPS::SP::Ph1::Transformer::setBaseVoltage, "base_voltage"_a)
		.def("connect", &CPS::SP::Ph1::Transformer::connect);
}

void addSPPh3Components(py::module_ mSPPh3) {
	py::class_<CPS::SP::Ph3::VoltageSource, std::shared_ptr<CPS::SP::Ph3::VoltageSource>, CPS::SimPowerComp<CPS::Complex>>(mSPPh3, "VoltageSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph3::VoltageSource::setParameters, "V_ref"_a)
		.def("connect", &CPS::SP::Ph3::VoltageSource::connect)
		.def_property("V_ref", createAttributeGetter<CPS::MatrixComp>("V_ref"), createAttributeSetter<CPS::MatrixComp>("V_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

	py::class_<CPS::SP::Ph3::Resistor, std::shared_ptr<CPS::SP::Ph3::Resistor>, CPS::SimPowerComp<CPS::Complex>>(mSPPh3, "Resistor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph3::Resistor::setParameters, "R"_a)
		.def("connect", &CPS::SP::Ph3::Resistor::connect);;

	py::class_<CPS::SP::Ph3::Capacitor, std::shared_ptr<CPS::SP::Ph3::Capacitor>, CPS::SimPowerComp<CPS::Complex>>(mSPPh3, "Capacitor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph3::Capacitor::setParameters, "C"_a)
		.def("connect", &CPS::SP::Ph3::Capacitor::connect);

	py::class_<CPS::SP::Ph3::Inductor, std::shared_ptr<CPS::SP::Ph3::Inductor>, CPS::SimPowerComp<CPS::Complex>>(mSPPh3, "Inductor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::SP::Ph3::Inductor::setParameters, "L"_a)
		.def("connect", &CPS::SP::Ph3::Inductor::connect);
}
