/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <SPComponents.h>

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
        .def("set_parameters", &CPS::SP::Ph1::VoltageSource::setParameters, "V_ref"_a, "f_src"_a = -1)
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
        .def("set_parameters", py::overload_cast<CPS::Complex, CPS::Real>(&CPS::SP::Ph1::NetworkInjection::setParameters), "V_ref"_a, "f_src"_a)
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

	py::class_<CPS::SP::Ph1::Switch, std::shared_ptr<CPS::SP::Ph1::Switch>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "Switch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::Switch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false)
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

	py::class_<CPS::SP::Ph1::varResSwitch, std::shared_ptr<CPS::SP::Ph1::varResSwitch>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "varResSwitch", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
        .def("set_parameters", &CPS::SP::Ph1::varResSwitch::setParameters, "open_resistance"_a, "closed_resistance"_a, "closed"_a = false)
		.def("open", &CPS::SP::Ph1::varResSwitch::open)
		.def("close", &CPS::SP::Ph1::varResSwitch::close)
		.def("set_init_parameters", &CPS::SP::Ph1::varResSwitch::setInitParameters, "time_step"_a)
		.def("connect", &CPS::SP::Ph1::varResSwitch::connect);

	py::class_<CPS::SP::Ph1::SynchronGeneratorTrStab, std::shared_ptr<CPS::SP::Ph1::SynchronGeneratorTrStab>, CPS::SimPowerComp<CPS::Complex>>(mSPPh1, "SynchronGeneratorTrStab", py::multiple_inheritance())
        .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("set_standard_parameters_PU", &CPS::SP::Ph1::SynchronGeneratorTrStab::setStandardParametersPU,
				"nom_power"_a, "nom_volt"_a, "nom_freq"_a, "Xpd"_a, "inertia"_a, "Rs"_a=0, "D"_a=0)
		.def("set_initial_values", &CPS::SP::Ph1::SynchronGeneratorTrStab::setInitialValues, "elec_power"_a, "mech_power"_a)
		.def("connect", &CPS::SP::Ph1::SynchronGeneratorTrStab::connect);
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
