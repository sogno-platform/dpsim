/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>

#include <cps/CSVReader.h>

namespace py = pybind11;
using namespace pybind11::literals;

template <typename T>
py::cpp_function createAttributeSetter(const std::string name) {
	return [name](CPS::IdentifiedObject &object, T &value) {
		object.attribute<T>(name)->set(value);
	};
}

template <typename T>
py::cpp_function createAttributeGetter(const std::string name) {
	return [name](CPS::IdentifiedObject &object) {
		return object.attribute<T>(name)->get();
	};
}

PYBIND11_MODULE(dpsimpy, m) {
    m.doc() = R"pbdoc(
        Pybind11 DPsim plugin
        -----------------------
        .. currentmodule:: dpsimpy
        .. autosummary::
           :toctree: _generate
    )pbdoc";

	py::enum_<CPS::Logger::Level>(m, "LogLevel")
		.value("trace", CPS::Logger::Level::trace)
		.value("debug", CPS::Logger::Level::debug)
		.value("info", CPS::Logger::Level::info)
		.value("warn", CPS::Logger::Level::warn)
		.value("err", CPS::Logger::Level::err)
		.value("critical", CPS::Logger::Level::critical)
		.value("off", CPS::Logger::Level::off);

	py::class_<CPS::Matrix>(m, "Matrix");
	py::class_<CPS::MatrixComp>(m, "MatrixComp");

	py::class_<CPS::Math>(m, "Math")
		.def_static("single_phase_variable_to_three_phase", &CPS::Math::singlePhaseVariableToThreePhase)
		.def_static("single_phase_parameter_to_three_phase", &CPS::Math::singlePhaseParameterToThreePhase);


    py::class_<DPsim::Simulation>(m, "Simulation")
	    .def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::off)
		.def("name", &DPsim::Simulation::name)
		.def("set_time_step", &DPsim::Simulation::setTimeStep)
		.def("set_final_time", &DPsim::Simulation::setFinalTime)
		.def("add_logger", &DPsim::Simulation::addLogger)
		.def("set_system", &DPsim::Simulation::setSystem)
		.def("run", &DPsim::Simulation::run)
		.def("set_solver", &DPsim::Simulation::setSolverType)
		.def("set_domain", &DPsim::Simulation::setDomain)
		.def("start", &DPsim::Simulation::start)
		.def("next", &DPsim::Simulation::next)
		.def("set_idobj_attr", static_cast<void (DPsim::Simulation::*)(const std::string&, const std::string&, CPS::Real)>(&DPsim::Simulation::setIdObjAttr))
		.def("set_idobj_attr", static_cast<void (DPsim::Simulation::*)(const std::string&, const std::string&, CPS::Complex)>(&DPsim::Simulation::setIdObjAttr))
		.def("get_real_idobj_attr", &DPsim::Simulation::getRealIdObjAttr, "obj"_a, "attr"_a, "row"_a = 0, "col"_a = 0)
		.def("get_comp_idobj_attr", &DPsim::Simulation::getComplexIdObjAttr, "obj"_a, "attr"_a, "row"_a = 0, "col"_a = 0)
		.def("add_interface", &DPsim::Simulation::addInterface, "interface"_a, "syncStart"_a = false)
		.def("export_attr", &DPsim::Simulation::exportIdObjAttr, "obj"_a, "attr"_a, "idx"_a, "modifier"_a, "row"_a = 0, "col"_a = 0)
		.def("import_attr", &DPsim::Simulation::importIdObjAttr, "obj"_a, "attr"_a, "idx"_a)
		.def("log_attr", &DPsim::Simulation::logIdObjAttr);

	py::class_<DPsim::RealTimeSimulation, DPsim::Simulation>(m, "RealTimeSimulation")
		.def(py::init<std::string, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::info)
		.def("name", &DPsim::RealTimeSimulation::name)
		.def("set_time_step", &DPsim::RealTimeSimulation::setTimeStep)
		.def("set_final_time", &DPsim::RealTimeSimulation::setFinalTime)
		.def("add_logger", &DPsim::RealTimeSimulation::addLogger)
		.def("set_system", &DPsim::RealTimeSimulation::setSystem)
		.def("run", static_cast<void (DPsim::RealTimeSimulation::*)(CPS::Int startIn)>(&DPsim::RealTimeSimulation::run))
		.def("set_solver", &DPsim::RealTimeSimulation::setSolverType)
		.def("set_domain", &DPsim::RealTimeSimulation::setDomain)
		.def("set_idobj_attr", static_cast<void (DPsim::RealTimeSimulation::*)(const std::string&, const std::string&, CPS::Real)>(&DPsim::Simulation::setIdObjAttr))
		.def("set_idobj_attr", static_cast<void (DPsim::RealTimeSimulation::*)(const std::string&, const std::string&, CPS::Complex)>(&DPsim::Simulation::setIdObjAttr))
		.def("get_real_idobj_attr", &DPsim::RealTimeSimulation::getRealIdObjAttr, "obj"_a, "attr"_a, "row"_a = 0, "col"_a= 0)
		.def("get_comp_idobj_attr", &DPsim::RealTimeSimulation::getComplexIdObjAttr, "obj"_a, "attr"_a, "row"_a = 0, "col"_a= 0)
		.def("add_interface", &DPsim::RealTimeSimulation::addInterface, "interface"_a, "syncStart"_a = false)
		.def("export_attr", &DPsim::RealTimeSimulation::exportIdObjAttr, "obj"_a, "attr"_a, "idx"_a, "modifier"_a, "row"_a = 0, "col"_a = 0)
		.def("log_attr", &DPsim::RealTimeSimulation::logIdObjAttr);


	py::class_<CPS::SystemTopology, std::shared_ptr<CPS::SystemTopology>>(m, "SystemTopology")
        .def(py::init<CPS::Real, CPS::TopologicalNode::List, CPS::IdentifiedObject::List>())
		.def(py::init<CPS::Real>())
		.def("add", &DPsim::SystemTopology::addComponent)
		.def("_repr_svg_", &DPsim::SystemTopology::render)
		.def("render_to_file", &DPsim::SystemTopology::renderToFile)
		.def_readwrite("nodes", &DPsim::SystemTopology::mNodes)
		.def("list_idobjects", &DPsim::SystemTopology::listIdObjects);

	py::class_<DPsim::Interface>(m, "Interface");

	py::class_<DPsim::DataLogger, std::shared_ptr<DPsim::DataLogger>>(m, "Logger")
        .def(py::init<std::string>())
		.def_static("set_log_dir", [](std::string dir) {
            CPS::Logger::setLogDir(dir);
        })
		.def("log_attribute", (void (DPsim::DataLogger::*)(const CPS::String &, const CPS::String &, CPS::IdentifiedObject::Ptr)) &DPsim::DataLogger::addAttribute);

	py::class_<CPS::IdentifiedObject, std::shared_ptr<CPS::IdentifiedObject>>(m, "IdentifiedObject")
		.def("name", &CPS::IdentifiedObject::name);

	py::enum_<CPS::AttributeBase::Modifier>(m, "AttrModifier")
		.value("real", CPS::AttributeBase::Modifier::real)
		.value("imag", CPS::AttributeBase::Modifier::imag)
		.value("mag", CPS::AttributeBase::Modifier::mag)
		.value("phase", CPS::AttributeBase::Modifier::phase);

	py::enum_<CPS::Domain>(m, "Domain")
		.value("SP", CPS::Domain::SP)
		.value("DP", CPS::Domain::DP)
		.value("EMT", CPS::Domain::EMT);

	py::enum_<CPS::PhaseType>(m, "PhaseType")
		.value("A", CPS::PhaseType::A)
		.value("B", CPS::PhaseType::B)
		.value("C", CPS::PhaseType::C)
		.value("ABC", CPS::PhaseType::ABC)
		.value("Single", CPS::PhaseType::Single);

	py::enum_<CPS::GeneratorType>(m, "GeneratorType")
		.value("PVNode", CPS::GeneratorType::PVNode)
		.value("TransientStability", CPS::GeneratorType::TransientStability)
		.value("IdealVoltageSource", CPS::GeneratorType::IdealVoltageSource)
		.value("None", CPS::GeneratorType::None);

	py::enum_<DPsim::Solver::Type>(m, "Solver")
		.value("MNA", DPsim::Solver::Type::MNA)
		.value("DAE", DPsim::Solver::Type::DAE)
		.value("NRP", DPsim::Solver::Type::NRP);

	py::enum_<CPS::CSVReader::Mode>(m, "CSVReaderMode")
		.value("AUTO", CPS::CSVReader::Mode::AUTO)
		.value("MANUAL", CPS::CSVReader::Mode::MANUAL);

	py::enum_<CPS::CSVReader::DataFormat>(m, "CSVReaderFormat")
		.value("HHMMSS", CPS::CSVReader::DataFormat::HHMMSS)
		.value("SECONDS", CPS::CSVReader::DataFormat::SECONDS)
		.value("HOURS", CPS::CSVReader::DataFormat::HOURS)
		.value("MINUTES", CPS::CSVReader::DataFormat::MINUTES);

	py::class_<CPS::CIM::Reader>(m, "CIMReader")
		.def(py::init<std::string, CPS::Logger::Level, CPS::Logger::Level>(), "name"_a, "loglevel"_a = CPS::Logger::Level::info, "comploglevel"_a = CPS::Logger::Level::off)
		.def("loadCIM", (CPS::SystemTopology (CPS::CIM::Reader::*)(CPS::Real, const std::list<CPS::String> &, CPS::Domain, CPS::PhaseType, CPS::GeneratorType)) &CPS::CIM::Reader::loadCIM);

	py::class_<CPS::CSVReader>(m, "CSVReader")
		.def(py::init<std::string, const std::string &, std::map<std::string, std::string> &, CPS::Logger::Level>())
		.def("assignLoadProfile", &CPS::CSVReader::assignLoadProfile);

	py::class_<CPS::TopologicalPowerComp, std::shared_ptr<CPS::TopologicalPowerComp>, CPS::IdentifiedObject>(m, "TopologicalPowerComp");
	py::class_<CPS::SimPowerComp<CPS::Complex>, std::shared_ptr<CPS::SimPowerComp<CPS::Complex>>, CPS::TopologicalPowerComp>(m, "SimPowerCompComplex");
	py::class_<CPS::SimPowerComp<CPS::Real>, std::shared_ptr<CPS::SimPowerComp<CPS::Real>>, CPS::TopologicalPowerComp>(m, "SimPowerCompReal");

	py::class_<CPS::TopologicalNode, std::shared_ptr<CPS::TopologicalNode>, CPS::IdentifiedObject>(m, "TopologicalNode");

	py::module mDP = m.def_submodule("dp", "dynamic phasor models");
	py::module mDPPh1 = mDP.def_submodule("ph1", "single phase dynamic phasor models");
	py::module mEMT = m.def_submodule("emt", "electromagnetic-transient models");
	py::module mEMTPh1 = mEMT.def_submodule("ph1", "single phase electromagnetic-transient models");
	py::module mEMTPh3 = mEMT.def_submodule("ph3", "three phase electromagnetic-transient models");
	py::module mSP = m.def_submodule("sp", "static phasor models");
	py::module mSPPh1 = mSP.def_submodule("ph1", "single phase static phasor models");
	py::module mSPPh3 = mSP.def_submodule("ph3", "three phase static phasor models");

	//DP-Components
    py::class_<CPS::DP::SimNode, std::shared_ptr<CPS::DP::SimNode>, CPS::TopologicalNode>(mDP, "SimNode", py::module_local())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::PhaseType>())
		.def(py::init<std::string, CPS::PhaseType, const std::vector<CPS::Complex>>())
		.def("set_initial_voltage", py::detail::overload_cast_impl<CPS::MatrixComp>()(&CPS::DP::SimNode::setInitialVoltage))
		.def("set_initial_voltage", py::detail::overload_cast_impl<CPS::Complex>()(&CPS::DP::SimNode::setInitialVoltage))
		.def("set_initial_voltage", py::detail::overload_cast_impl<CPS::Complex, int>()(&CPS::DP::SimNode::setInitialVoltage))
		.def_readonly_static("gnd", &CPS::DP::SimNode::GND);

	py::class_<CPS::DP::Ph1::VoltageSource, std::shared_ptr<CPS::DP::Ph1::VoltageSource>, CPS::SimPowerComp<CPS::Complex>>(mDPPh1, "VoltageSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::DP::Ph1::VoltageSource::setParameters, "V_ref"_a, "f_src"_a=-1)
		.def("connect", &CPS::DP::Ph1::VoltageSource::connect)
		.def_property("V_ref", createAttributeGetter<CPS::Complex>("V_ref"), createAttributeSetter<CPS::Complex>("V_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

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

	//EMT Components
	py::class_<CPS::EMT::SimNode, std::shared_ptr<CPS::EMT::SimNode>, CPS::TopologicalNode>(mEMT, "SimNode", py::module_local())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::PhaseType>())
		.def(py::init<std::string, CPS::PhaseType, const std::vector<CPS::Complex>>())
		.def("set_initial_voltage", py::detail::overload_cast_impl<CPS::MatrixComp>()(&CPS::EMT::SimNode::setInitialVoltage))
		.def("set_initial_voltage", py::detail::overload_cast_impl<CPS::Complex>()(&CPS::EMT::SimNode::setInitialVoltage))
		.def("set_initial_voltage", py::detail::overload_cast_impl<CPS::Complex, int>()(&CPS::EMT::SimNode::setInitialVoltage))
		.def_readonly_static("gnd", &CPS::EMT::SimNode::GND);

	//EMT Ph1 Components
	py::class_<CPS::EMT::Ph1::CurrentSource, std::shared_ptr<CPS::EMT::Ph1::CurrentSource>, CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "CurrentSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph1::CurrentSource::setParameters, "I_ref"_a, "f_src"_a = -1)
		.def("connect", &CPS::EMT::Ph1::CurrentSource::connect)
		.def_property("I_ref", createAttributeGetter<CPS::Complex>("I_ref"), createAttributeSetter<CPS::Complex>("I_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

	py::class_<CPS::EMT::Ph1::VoltageSource, std::shared_ptr<CPS::EMT::Ph1::VoltageSource>, CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "VoltageSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph1::VoltageSource::setParameters, "V_ref"_a, "f_src"_a = -1)
		.def("connect", &CPS::EMT::Ph1::VoltageSource::connect)
		.def_property("V_ref", createAttributeGetter<CPS::Complex>("V_ref"), createAttributeSetter<CPS::Complex>("V_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

	py::class_<CPS::EMT::Ph1::Resistor, std::shared_ptr<CPS::EMT::Ph1::Resistor>, CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Resistor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph1::Resistor::setParameters, "R"_a)
		.def("connect", &CPS::EMT::Ph1::Resistor::connect)
		.def_property("R", createAttributeGetter<CPS::Real>("R"), createAttributeSetter<CPS::Real>("R"));

	py::class_<CPS::EMT::Ph1::Capacitor, std::shared_ptr<CPS::EMT::Ph1::Capacitor>, CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Capacitor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph1::Capacitor::setParameters, "C"_a)
		.def("connect", &CPS::EMT::Ph1::Capacitor::connect)
		.def_property("C", createAttributeGetter<CPS::Real>("C"), createAttributeSetter<CPS::Real>("C"));

	py::class_<CPS::EMT::Ph1::Inductor, std::shared_ptr<CPS::EMT::Ph1::Inductor>, CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Inductor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph1::Inductor::setParameters, "L"_a)
		.def("connect", &CPS::EMT::Ph1::Inductor::connect)
		.def_property("L", createAttributeGetter<CPS::Real>("L"), createAttributeSetter<CPS::Real>("L"));

	//EMT Ph3 Components
	py::class_<CPS::EMT::Ph3::VoltageSource, std::shared_ptr<CPS::EMT::Ph3::VoltageSource>, CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "VoltageSource", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph3::VoltageSource::setParameters, "V_ref"_a, "f_src"_a = -1)
		.def("connect", &CPS::EMT::Ph3::VoltageSource::connect)
		.def_property("V_ref", createAttributeGetter<CPS::MatrixComp>("V_ref"), createAttributeSetter<CPS::MatrixComp>("V_ref"))
		.def_property("f_src", createAttributeGetter<CPS::Real>("f_src"), createAttributeSetter<CPS::Real>("f_src"));

	py::class_<CPS::EMT::Ph3::Resistor, std::shared_ptr<CPS::EMT::Ph3::Resistor>, CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Resistor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph3::Resistor::setParameters, "R"_a)
		.def("connect", &CPS::EMT::Ph3::Resistor::connect);;

	py::class_<CPS::EMT::Ph3::Capacitor, std::shared_ptr<CPS::EMT::Ph3::Capacitor>, CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Capacitor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph3::Capacitor::setParameters, "C"_a)
		.def("connect", &CPS::EMT::Ph3::Capacitor::connect);

	py::class_<CPS::EMT::Ph3::Inductor, std::shared_ptr<CPS::EMT::Ph3::Inductor>, CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Inductor", py::multiple_inheritance())
        .def(py::init<std::string>())
		.def(py::init<std::string, CPS::Logger::Level>())
        .def("set_parameters", &CPS::EMT::Ph3::Inductor::setParameters, "L"_a)
		.def("connect", &CPS::EMT::Ph3::Inductor::connect);


	//SP Components
	mSP.attr("SimNode") = mDP.attr("SimNode");

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

	//SP Ph3 Components
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


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
