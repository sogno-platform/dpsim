/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iomanip>

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>

#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>

#include <cps/CSVReader.h>

#include <DPComponents.h>
#include <EMTComponents.h>
#include <SPComponents.h>
#include <SignalComponents.h>
#include <BaseComponents.h>
#include <Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(dpsimpy, m) {
    m.doc() = R"pbdoc(
	DPsim Python bindings
	-----------------------
	The Python bindings provide access to most of the DPsim features implemented in C++.
	It is possible to run powerflow, quasi-static, dynamic phasor and electromagnetic transient simulations
	and to parameterize all components of the network from Python.
    )pbdoc";

	py::enum_<CPS::Logger::Level>(m, "LogLevel")
		.value("trace", CPS::Logger::Level::trace)
		.value("debug", CPS::Logger::Level::debug)
		.value("info", CPS::Logger::Level::info)
		.value("warn", CPS::Logger::Level::warn)
		.value("err", CPS::Logger::Level::err)
		.value("critical", CPS::Logger::Level::critical)
		.value("off", CPS::Logger::Level::off);

	py::class_<CPS::Math>(m, "Math")
		.def_static("single_phase_variable_to_three_phase", &CPS::Math::singlePhaseVariableToThreePhase)
		.def_static("single_phase_parameter_to_three_phase", &CPS::Math::singlePhaseParameterToThreePhase)
		.def_static("single_phase_power_to_three_phase", &CPS::Math::singlePhasePowerToThreePhase);

	m.attr("RMS3PH_TO_PEAK1PH") = RMS3PH_TO_PEAK1PH;
	m.attr("PEAK1PH_TO_RMS3PH") = PEAK1PH_TO_RMS3PH;

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
		.def("export_attr", py::overload_cast<const CPS::String&, const CPS::String&, CPS::UInt, CPS::AttributeBase::Modifier, CPS::UInt, CPS::UInt, DPsim::Interface*>(&DPsim::Simulation::exportIdObjAttr), "obj"_a, "attr"_a, "idx"_a, "modifier"_a, "row"_a = 0, "col"_a = 0, "intf"_a = nullptr)
		.def("export_attr", py::overload_cast<const CPS::String&, const CPS::String&, CPS::UInt, CPS::UInt, CPS::UInt, CPS::Complex, DPsim::Interface*>(&DPsim::Simulation::exportIdObjAttr), "obj"_a, "attr"_a, "idx"_a, "row"_a = 0, "col"_a = 0, "scale"_a = CPS::Complex(1, 0), "intf"_a = nullptr)
		.def("import_attr", &DPsim::Simulation::importIdObjAttr, "obj"_a, "attr"_a, "idx"_a, "intf"_a = nullptr)
		.def("log_attr", &DPsim::Simulation::logIdObjAttr)
		.def("do_init_from_nodes_and_terminals", &DPsim::Simulation::doInitFromNodesAndTerminals)
		.def("do_system_matrix_recomputation", &DPsim::Simulation::doSystemMatrixRecomputation)
		.def("do_steady_state_init", &DPsim::Simulation::doSteadyStateInit)
		.def("do_frequency_parallelization", &DPsim::Simulation::doFrequencyParallelization)
		.def("set_tearing_components", &DPsim::Simulation::setTearingComponents)
		.def("add_event", &DPsim::Simulation::addEvent);

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
		.def("log_attr", &DPsim::RealTimeSimulation::logIdObjAttr);

	py::class_<CPS::SystemTopology, std::shared_ptr<CPS::SystemTopology>>(m, "SystemTopology")
        .def(py::init<CPS::Real, CPS::TopologicalNode::List, CPS::IdentifiedObject::List>())
		.def(py::init<CPS::Real, CPS::Matrix, CPS::TopologicalNode::List, CPS::IdentifiedObject::List>())
		.def(py::init<CPS::Real>())
		.def("add", &DPsim::SystemTopology::addComponent)
		.def("add", &DPsim::SystemTopology::addComponents)
		.def("node", py::overload_cast<const CPS::String&>(&DPsim::SystemTopology::node<CPS::TopologicalNode>))
		.def("node", py::overload_cast<CPS::UInt>(&DPsim::SystemTopology::node<CPS::TopologicalNode>))
		.def("connect_component", py::overload_cast<CPS::SimPowerComp<CPS::Real>::Ptr, CPS::SimNode<CPS::Real>::List>(&DPsim::SystemTopology::connectComponentToNodes<CPS::Real>))
		.def("connect_component", py::overload_cast<CPS::SimPowerComp<CPS::Complex>::Ptr, CPS::SimNode<CPS::Complex>::List>(&DPsim::SystemTopology::connectComponentToNodes<CPS::Complex>))
		.def("component", &DPsim::SystemTopology::component<CPS::TopologicalPowerComp>)
		.def("add_tear_component", &DPsim::SystemTopology::addTearComponent)
		.def("_repr_svg_", &DPsim::SystemTopology::render)
		.def("render_to_file", &DPsim::SystemTopology::renderToFile)
		.def_readwrite("nodes", &DPsim::SystemTopology::mNodes)
		.def_readwrite("components", &DPsim::SystemTopology::mComponents)
		.def_readonly("tear_components", &DPsim::SystemTopology::mTearComponents)
		.def("list_idobjects", &DPsim::SystemTopology::listIdObjects);

	py::class_<DPsim::Interface>(m, "Interface");

	py::class_<DPsim::DataLogger, std::shared_ptr<DPsim::DataLogger>>(m, "Logger")
        .def(py::init<std::string>())
		.def_static("set_log_dir", &CPS::Logger::setLogDir)
		.def_static("get_log_dir", &CPS::Logger::logDir)
		.def("log_attribute", (void (DPsim::DataLogger::*)(const CPS::String &, const CPS::String &, CPS::IdentifiedObject::Ptr)) &DPsim::DataLogger::addAttribute)
		.def("log_attribute", (void (DPsim::DataLogger::*)(const CPS::String &, const CPS::String &, CPS::IdentifiedObject::Ptr, CPS::UInt, CPS::UInt)) &DPsim::DataLogger::addAttribute)
		.def("log_attribute", (void (DPsim::DataLogger::*)(const std::vector<CPS::String> &, const CPS::String &, CPS::IdentifiedObject::Ptr)) &DPsim::DataLogger::addAttribute);

	py::class_<CPS::IdentifiedObject, std::shared_ptr<CPS::IdentifiedObject>>(m, "IdentifiedObject")
		.def("name", &CPS::IdentifiedObject::name)
		.def("print_attribute_list", &printAttributes)
		.def("print_attribute", &printAttribute, "attribute_name"_a)
		.def("__str__", &getAttributeList);

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

	py::enum_<CPS::PowerflowBusType>(m, "PowerflowBusType")
		.value("PV", CPS::PowerflowBusType::PV)
		.value("PQ", CPS::PowerflowBusType::PQ)
		.value("VD", CPS::PowerflowBusType::VD)
		.value("None", CPS::PowerflowBusType::None);

	py::enum_<CPS::GeneratorType>(m, "GeneratorType")
		.value("PVNode", CPS::GeneratorType::PVNode)
		.value("TransientStability", CPS::GeneratorType::TransientStability)
		.value("IdealVoltageSource", CPS::GeneratorType::IdealVoltageSource)
		.value("NONE", CPS::GeneratorType::None);

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
		.def("loadCIM", (CPS::SystemTopology (CPS::CIM::Reader::*)(CPS::Real, const std::list<CPS::String> &, CPS::Domain, CPS::PhaseType, CPS::GeneratorType)) &CPS::CIM::Reader::loadCIM)
		.def("init_dynamic_system_topology_with_powerflow", &CPS::CIM::Reader::initDynamicSystemTopologyWithPowerflow);

	py::class_<CPS::CSVReader>(m, "CSVReader")
		.def(py::init<std::string, const std::string &, std::map<std::string, std::string> &, CPS::Logger::Level>())
		.def("assignLoadProfile", &CPS::CSVReader::assignLoadProfile);

	py::class_<CPS::TopologicalPowerComp, std::shared_ptr<CPS::TopologicalPowerComp>, CPS::IdentifiedObject>(m, "TopologicalPowerComp");
	py::class_<CPS::SimPowerComp<CPS::Complex>, std::shared_ptr<CPS::SimPowerComp<CPS::Complex>>, CPS::TopologicalPowerComp>(m, "SimPowerCompComplex")
		.def("connect", &CPS::SimPowerComp<CPS::Complex>::connect)
		.def("set_intf_current", &CPS::SimPowerComp<CPS::Complex>::setIntfCurrent)
		.def("set_intf_voltage", &CPS::SimPowerComp<CPS::Complex>::setIntfVoltage);
	py::class_<CPS::SimPowerComp<CPS::Real>, std::shared_ptr<CPS::SimPowerComp<CPS::Real>>, CPS::TopologicalPowerComp>(m, "SimPowerCompReal")
		.def("connect", &CPS::SimPowerComp<CPS::Real>::connect)
		.def("set_intf_current", &CPS::SimPowerComp<CPS::Real>::setIntfCurrent)
		.def("set_intf_voltage", &CPS::SimPowerComp<CPS::Real>::setIntfVoltage);
	py::class_<CPS::TopologicalNode, std::shared_ptr<CPS::TopologicalNode>, CPS::IdentifiedObject>(m, "TopologicalNode")
		.def("initial_single_voltage", &CPS::TopologicalNode::initialSingleVoltage, "phase_type"_a = CPS::PhaseType::Single);

	//Events
	py::module mEvent = m.def_submodule("event", "events");
	py::class_<DPsim::Event, std::shared_ptr<DPsim::Event>>(mEvent, "Event");
	py::class_<DPsim::SwitchEvent, std::shared_ptr<DPsim::SwitchEvent>, DPsim::Event>(mEvent, "SwitchEvent", py::multiple_inheritance())
		.def(py::init<CPS::Real,const std::shared_ptr<CPS::Base::Ph1::Switch>,CPS::Bool>());
	py::class_<DPsim::SwitchEvent3Ph, std::shared_ptr<DPsim::SwitchEvent3Ph>, DPsim::Event>(mEvent, "SwitchEvent3Ph", py::multiple_inheritance())
		.def(py::init<CPS::Real,const std::shared_ptr<CPS::Base::Ph3::Switch>,CPS::Bool>());

	//Components
	py::module mBase = m.def_submodule("base", "base models");
	addBaseComponents(mBase);

	py::module mDP = m.def_submodule("dp", "dynamic phasor models");
	addDPComponents(mDP);

	py::module mEMT = m.def_submodule("emt", "electromagnetic-transient models");
	addEMTComponents(mEMT);

	py::module mSP = m.def_submodule("sp", "static phasor models");
	mSP.attr("SimNode") = mDP.attr("SimNode");
	addSPComponents(mSP);

	py::module mSignal = m.def_submodule("signal", "signal models");
	addSignalComponents(mSignal);


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
