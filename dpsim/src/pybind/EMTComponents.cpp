/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim-models/CSVReader.h>
#include <dpsim-models/IdentifiedObject.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim/Simulation.h>
#include <dpsim/pybind/EMTComponents.h>
#include <dpsim/pybind/Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addEMTComponents(py::module_ mEMT) {
  py::class_<CPS::EMT::SimNode, std::shared_ptr<CPS::EMT::SimNode>,
             CPS::TopologicalNode>(mEMT, "SimNode", py::module_local())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::PhaseType>())
      .def(py::init<std::string, CPS::PhaseType,
                    const std::vector<CPS::Complex>>())
      .def("set_initial_voltage",
           py::overload_cast<CPS::MatrixComp>(
               &CPS::EMT::SimNode::setInitialVoltage, py::const_))
      .def("set_initial_voltage",
           py::overload_cast<CPS::Complex>(
               &CPS::EMT::SimNode::setInitialVoltage, py::const_))
      .def("set_initial_voltage",
           py::overload_cast<CPS::Complex, int>(
               &CPS::EMT::SimNode::setInitialVoltage, py::const_))
      .def_readonly_static("gnd", &CPS::EMT::SimNode::GND);

  py::module mEMTPh1 = mEMT.def_submodule(
      "ph1", "single phase electromagnetic-transient models");
  addEMTPh1Components(mEMTPh1);
  py::module mEMTPh3 =
      mEMT.def_submodule("ph3", "three phase electromagnetic-transient models");
  addEMTPh3Components(mEMTPh3);
}

void addEMTPh1Components(py::module_ mEMTPh1) {
  py::class_<CPS::EMT::Ph1::CurrentSource,
             std::shared_ptr<CPS::EMT::Ph1::CurrentSource>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "CurrentSource",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::CurrentSource::setParameters,
           "I_ref"_a, "f_src"_a = -1)
      .def("connect", &CPS::EMT::Ph1::CurrentSource::connect)
      .def_property("I_ref", createAttributeGetter<CPS::Complex>("I_ref"),
                    createAttributeSetter<CPS::Complex>("I_ref"))
      .def_property("f_src", createAttributeGetter<CPS::Real>("f_src"),
                    createAttributeSetter<CPS::Real>("f_src"));

  py::class_<CPS::EMT::Ph1::VoltageSource,
             std::shared_ptr<CPS::EMT::Ph1::VoltageSource>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "VoltageSource",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::VoltageSource::setParameters,
           "V_ref"_a, "f_src"_a = -1)
      .def("connect", &CPS::EMT::Ph1::VoltageSource::connect)
      .def_property("V_ref", createAttributeGetter<CPS::Complex>("V_ref"),
                    createAttributeSetter<CPS::Complex>("V_ref"))
      .def_property("f_src", createAttributeGetter<CPS::Real>("f_src"),
                    createAttributeSetter<CPS::Real>("f_src"));

  py::class_<CPS::EMT::Ph1::Resistor, std::shared_ptr<CPS::EMT::Ph1::Resistor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Resistor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::Resistor::setParameters, "R"_a)
      .def("connect", &CPS::EMT::Ph1::Resistor::connect)
      .def_property("R", createAttributeGetter<CPS::Real>("R"),
                    createAttributeSetter<CPS::Real>("R"));

  py::class_<CPS::EMT::Ph1::Capacitor,
             std::shared_ptr<CPS::EMT::Ph1::Capacitor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Capacitor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::Capacitor::setParameters, "C"_a)
      .def("connect", &CPS::EMT::Ph1::Capacitor::connect)
      .def_property("C", createAttributeGetter<CPS::Real>("C"),
                    createAttributeSetter<CPS::Real>("C"));

  py::class_<CPS::EMT::Ph1::Inductor, std::shared_ptr<CPS::EMT::Ph1::Inductor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Inductor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::Inductor::setParameters, "L"_a)
      .def("connect", &CPS::EMT::Ph1::Inductor::connect)
      .def_property("L", createAttributeGetter<CPS::Real>("L"),
                    createAttributeSetter<CPS::Real>("L"));

  py::class_<CPS::EMT::Ph1::Switch, std::shared_ptr<CPS::EMT::Ph1::Switch>,
             CPS::SimPowerComp<CPS::Real>, CPS::Base::Ph1::Switch>(
      mEMTPh1, "Switch", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters", &CPS::EMT::Ph1::Switch::setParameters,
           "open_resistance"_a, "closed_resistance"_a,
           "closed"_a = false) // cppcheck-suppress assignBoolToPointer
      .def("open", &CPS::EMT::Ph1::Switch::open)
      .def("close", &CPS::EMT::Ph1::Switch::close)
      .def("connect", &CPS::EMT::Ph1::Switch::connect);

  py::class_<CPS::EMT::Ph1::ExponentialDiode,
             std::shared_ptr<CPS::EMT::Ph1::ExponentialDiode>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "ExponentialDiode",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::ExponentialDiode::setParameters,
           "I_S"_a, "V_T"_a)
      .def("connect", &CPS::EMT::Ph1::ExponentialDiode::connect);
}

void addEMTPh3Components(py::module_ mEMTPh3) {
  py::class_<CPS::EMT::Ph3::VoltageSource,
             std::shared_ptr<CPS::EMT::Ph3::VoltageSource>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "VoltageSource",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           py::overload_cast<CPS::MatrixComp, CPS::Real>(
               &CPS::EMT::Ph3::VoltageSource::setParameters),
           "V_ref"_a, "f_src"_a = 50)
      .def("connect", &CPS::EMT::Ph3::VoltageSource::connect)
      .def_property("V_ref", createAttributeGetter<CPS::MatrixComp>("V_ref"),
                    createAttributeSetter<CPS::MatrixComp>("V_ref"))
      .def_property("f_src", createAttributeGetter<CPS::Real>("f_src"),
                    createAttributeSetter<CPS::Real>("f_src"));

  py::class_<CPS::EMT::Ph3::CurrentSource,
             std::shared_ptr<CPS::EMT::Ph3::CurrentSource>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "CurrentSource",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           py::overload_cast<CPS::MatrixComp, CPS::Real>(
               &CPS::EMT::Ph3::CurrentSource::setParameters),
           "I_ref"_a, "f_src"_a = 50)
      .def("connect", &CPS::EMT::Ph3::VoltageSource::connect)
      .def_property("I_ref", createAttributeGetter<CPS::MatrixComp>("I_ref"),
                    createAttributeSetter<CPS::MatrixComp>("V_ref"))
      .def_property("f_src", createAttributeGetter<CPS::Real>("f_src"),
                    createAttributeSetter<CPS::Real>("f_src"));

  py::class_<CPS::EMT::Ph3::Resistor, std::shared_ptr<CPS::EMT::Ph3::Resistor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Resistor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph3::Resistor::setParameters, "R"_a)
      .def("connect", &CPS::EMT::Ph3::Resistor::connect);

  py::class_<CPS::EMT::Ph3::Capacitor,
             std::shared_ptr<CPS::EMT::Ph3::Capacitor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Capacitor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph3::Capacitor::setParameters, "C"_a)
      .def("connect", &CPS::EMT::Ph3::Capacitor::connect);

  py::class_<CPS::EMT::Ph3::Inductor, std::shared_ptr<CPS::EMT::Ph3::Inductor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Inductor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph3::Inductor::setParameters, "L"_a)
      .def("connect", &CPS::EMT::Ph3::Inductor::connect);

  py::class_<CPS::EMT::Ph3::NetworkInjection,
             std::shared_ptr<CPS::EMT::Ph3::NetworkInjection>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "NetworkInjection",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters",
           py::overload_cast<CPS::MatrixComp, CPS::Real>(
               &CPS::EMT::Ph3::NetworkInjection::setParameters),
           "V_ref"_a, "f_src"_a = 50)
      .def("connect", &CPS::EMT::Ph3::NetworkInjection::connect);

  py::class_<CPS::EMT::Ph3::PiLine, std::shared_ptr<CPS::EMT::Ph3::PiLine>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "PiLine",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters", &CPS::EMT::Ph3::PiLine::setParameters,
           "series_resistance"_a, "series_inductance"_a,
           "parallel_capacitance"_a = zeroMatrix(3),
           "parallel_conductance"_a = zeroMatrix(3))
      .def("connect", &CPS::EMT::Ph3::PiLine::connect);

  py::class_<CPS::EMT::Ph3::RXLoad, std::shared_ptr<CPS::EMT::Ph3::RXLoad>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "RXLoad",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters", &CPS::EMT::Ph3::RXLoad::setParameters,
           "active_power"_a, "reactive_power"_a, "volt"_a)
      .def("connect", &CPS::EMT::Ph3::RXLoad::connect);

  py::class_<CPS::EMT::Ph3::Switch, std::shared_ptr<CPS::EMT::Ph3::Switch>,
             CPS::SimPowerComp<CPS::Real>, CPS::Base::Ph3::Switch>(
      mEMTPh3, "Switch", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters", &CPS::EMT::Ph3::Switch::setParameters,
           "open_resistance"_a, "closed_resistance"_a,
           // cppcheck-suppress assignBoolToPointer
           "closed"_a = false)
      .def("open", &CPS::EMT::Ph3::Switch::openSwitch)
      .def("close", &CPS::EMT::Ph3::Switch::closeSwitch)
      .def("connect", &CPS::EMT::Ph3::Switch::connect);

  py::class_<CPS::EMT::Ph3::SynchronGeneratorDQTrapez,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQTrapez>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "SynchronGeneratorDQTrapez",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters_operational_per_unit",
           &CPS::EMT::Ph3::SynchronGeneratorDQTrapez::
               setParametersOperationalPerUnit,
           "nom_power"_a, "nom_volt"_a, "nom_freq"_a, "pole_number"_a,
           "nom_field_cur"_a, "Rs"_a, "Ld"_a, "Lq"_a, "Ld_t"_a, "Lq_t"_a,
           "Ld_s"_a, "Lq_s"_a, "Ll"_a, "Td0_t"_a, "Tq0_t"_a, "Td0_s"_a,
           "Tq0_s"_a, "inertia"_a)
      .def("set_parameters_fundamental_per_unit",
           &CPS::EMT::Ph3::SynchronGeneratorDQTrapez::
               setParametersFundamentalPerUnit,
           "nom_power"_a, "nom_volt"_a, "nom_freq"_a, "pole_number"_a,
           "nom_field_cur"_a, "Rs"_a, "Ll"_a, "Lmd"_a, "Lmq"_a, "Rfd"_a,
           "Llfd"_a, "Rkd"_a, "Llkd"_a, "Rkq1"_a, "Llkq1"_a, "Rkq2"_a,
           "Llkq2"_a, "inertia"_a, "init_active_power"_a,
           "init_reactive_power"_a, "init_terminal_volt"_a, "init_volt_angle"_a,
           "init_mech_power"_a)
      .def("apply_parameters_from_json",
           [](std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQTrapez> syngen,
              const CPS::String json) {
             DPsim::Utils::applySynchronousGeneratorParametersFromJson(
                 json::parse(json), syngen);
           })
      .def("set_initial_values",
           &CPS::EMT::Ph3::SynchronGeneratorDQTrapez::setInitialValues,
           "init_active_power"_a, "init_reactive_power"_a,
           "init_terminal_volt"_a, "init_volt_angle"_a, "init_mech_power"_a);

  py::class_<CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR,
             std::shared_ptr<CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>,
             CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>>(
      mEMTPh3, "ReducedOrderSynchronGeneratorVBR", py::multiple_inheritance());

  py::class_<CPS::EMT::Ph3::SynchronGenerator3OrderVBR,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGenerator3OrderVBR>,
             CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>(
      mEMTPh3, "SynchronGenerator3OrderVBR", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_operational_parameters_per_unit",
           py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real>(
               &CPS::EMT::Ph3::SynchronGenerator3OrderVBR::
                   setOperationalParametersPerUnit),
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a,
           "Lq"_a, "L0"_a, "Ld_t"_a, "Td0_t"_a)
      .def("connect", &CPS::EMT::Ph3::SynchronGenerator3OrderVBR::connect);

  py::class_<CPS::EMT::Ph3::SynchronGenerator4OrderVBR,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGenerator4OrderVBR>,
             CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>(
      mEMTPh3, "SynchronGenerator4OrderVBR", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_operational_parameters_per_unit",
           py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real>(
               &CPS::EMT::Ph3::SynchronGenerator4OrderVBR::
                   setOperationalParametersPerUnit),
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a,
           "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a)
      .def("connect", &CPS::EMT::Ph3::SynchronGenerator4OrderVBR::connect);

  py::class_<CPS::EMT::Ph3::SynchronGenerator5OrderVBR,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGenerator5OrderVBR>,
             CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>(
      mEMTPh3, "SynchronGenerator5OrderVBR", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_operational_parameters_per_unit",
           py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real>(
               &CPS::EMT::Ph3::SynchronGenerator5OrderVBR::
                   setOperationalParametersPerUnit),
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a,
           "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a,
           "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a)
      .def("connect", &CPS::EMT::Ph3::SynchronGenerator5OrderVBR::connect);

  py::class_<CPS::EMT::Ph3::SynchronGenerator6aOrderVBR,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGenerator6aOrderVBR>,
             CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>(
      mEMTPh3, "SynchronGenerator6aOrderVBR", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_operational_parameters_per_unit",
           py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real>(
               &CPS::EMT::Ph3::SynchronGenerator6aOrderVBR::
                   setOperationalParametersPerUnit),
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a,
           "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a,
           "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a)
      .def("connect", &CPS::EMT::Ph3::SynchronGenerator6aOrderVBR::connect);

  py::class_<CPS::EMT::Ph3::SynchronGenerator6bOrderVBR,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGenerator6bOrderVBR>,
             CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>(
      mEMTPh3, "SynchronGenerator6bOrderVBR", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_operational_parameters_per_unit",
           py::overload_cast<CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real,
                             CPS::Real, CPS::Real, CPS::Real, CPS::Real>(
               &CPS::EMT::Ph3::SynchronGenerator6bOrderVBR::
                   setOperationalParametersPerUnit),
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a, "H"_a, "Ld"_a,
           "Lq"_a, "L0"_a, "Ld_t"_a, "Lq_t"_a, "Td0_t"_a, "Tq0_t"_a, "Ld_s"_a,
           "Lq_s"_a, "Td0_s"_a, "Tq0_s"_a, "Taa"_a = 0)
      .def("connect", &CPS::EMT::Ph3::SynchronGenerator6bOrderVBR::connect);

#ifdef WITH_SUNDIALS

  py::class_<CPS::EMT::Ph3::SynchronGeneratorDQODE,
             std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQODE>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "SynchronGeneratorDQODE",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters_operational_per_unit",
           &CPS::EMT::Ph3::SynchronGeneratorDQODE::
               setParametersOperationalPerUnit,
           "nom_power"_a, "nom_volt"_a, "nom_freq"_a, "pole_number"_a,
           "nom_field_cur"_a, "Rs"_a, "Ld"_a, "Lq"_a, "Ld_t"_a, "Lq_t"_a,
           "Ld_s"_a, "Lq_s"_a, "Ll"_a, "Td0_t"_a, "Tq0_t"_a, "Td0_s"_a,
           "Tq0_s"_a, "inertia"_a)
      .def("set_parameters_fundamental_per_unit",
           &CPS::EMT::Ph3::SynchronGeneratorDQODE::
               setParametersFundamentalPerUnit,
           "nom_power"_a, "nom_volt"_a, "nom_freq"_a, "pole_number"_a,
           "nom_field_cur"_a, "Rs"_a, "Ll"_a, "Lmd"_a, "Lmq"_a, "Rfd"_a,
           "Llfd"_a, "Rkd"_a, "Llkd"_a, "Rkq1"_a, "Llkq1"_a, "Rkq2"_a,
           "Llkq2"_a, "inertia"_a, "init_active_power"_a,
           "init_reactive_power"_a, "init_terminal_volt"_a, "init_volt_angle"_a,
           "init_mech_power"_a)
      .def("apply_parameters_from_json",
           [](std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQODE> syngen,
              const CPS::String json) {
             DPsim::Utils::applySynchronousGeneratorParametersFromJson(
                 json::parse(json), syngen);
           })
      .def("set_initial_values",
           &CPS::EMT::Ph3::SynchronGeneratorDQODE::setInitialValues,
           "init_active_power"_a, "init_reactive_power"_a,
           "init_terminal_volt"_a, "init_volt_angle"_a, "init_mech_power"_a);

#endif

  py::class_<CPS::EMT::Ph3::AvVoltageSourceInverterDQ,
             std::shared_ptr<CPS::EMT::Ph3::AvVoltageSourceInverterDQ>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "AvVoltageSourceInverterDQ",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool>(),
           "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off,
           // cppcheck-suppress assignBoolToPointer
           "with_trafo"_a = false)
      .def("set_parameters",
           &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::setParameters,
           "sys_omega"_a, "sys_volt_nom"_a, "p_ref"_a, "q_ref"_a)
      .def("set_filter_parameters",
           &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::setFilterParameters,
           "Lf"_a, "Cf"_a, "Rf"_a, "Rc"_a)
      .def("set_controller_parameters",
           &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::setControllerParameters,
           "Kp_pll"_a, "Ki_pll"_a, "Kp_power_ctrl"_a, "Ki_power_ctrl"_a,
           "Kp_curr_ctrl"_a, "Ki_curr_ctrl"_a, "omega_cutoff"_a)
      .def("set_transformer_parameters",
           &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::setTransformerParameters,
           "nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "rated_power"_a,
           "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a,
           "omega"_a)
      .def("set_initial_state_values",
           &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::setInitialStateValues,
           "p_init"_a, "q_init"_a, "phi_d_init"_a, "phi_q_init"_a,
           "gamma_d_init"_a, "gamma_q_init"_a)
      .def("with_control",
           &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::withControl)
      .def("connect", &CPS::EMT::Ph3::AvVoltageSourceInverterDQ::connect);

  py::class_<CPS::EMT::Ph3::Transformer,
             std::shared_ptr<CPS::EMT::Ph3::Transformer>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Transformer",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool>(),
           "uid"_a, "name"_a, "loglevel"_a = CPS::Logger::Level::off,
           // cppcheck-suppress assignBoolToPointer
           "with_resistive_losses"_a = false)
      .def("set_parameters", &CPS::EMT::Ph3::Transformer::setParameters,
           "nom_voltage_end_1"_a, "nom_voltage_end_2"_a, "rated_power"_a,
           "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a)
      .def("connect", &CPS::EMT::Ph3::Transformer::connect);

  py::class_<CPS::EMT::Ph3::SeriesResistor,
             std::shared_ptr<CPS::EMT::Ph3::SeriesResistor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "SeriesResistor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph3::SeriesResistor::setParameters,
           "R"_a)
      .def("connect", &CPS::EMT::Ph3::SeriesResistor::connect);

  py::class_<CPS::EMT::Ph3::SeriesSwitch,
             std::shared_ptr<CPS::EMT::Ph3::SeriesSwitch>,
             CPS::SimPowerComp<CPS::Real>, CPS::Base::Ph1::Switch>(
      mEMTPh3, "SeriesSwitch", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters", &CPS::EMT::Ph3::SeriesSwitch::setParameters,
           "open_resistance"_a, "closed_resistance"_a,
           // cppcheck-suppress assignBoolToPointer
           "closed"_a = false)
      .def("open", &CPS::EMT::Ph3::SeriesSwitch::open)
      .def("close", &CPS::EMT::Ph3::SeriesSwitch::close)
      .def("connect", &CPS::EMT::Ph3::SeriesSwitch::connect);

  py::class_<CPS::EMT::Ph3::SSN::Full_Serial_RLC,
             std::shared_ptr<CPS::EMT::Ph3::SSN::Full_Serial_RLC>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Full_Serial_RLC",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::EMT::Ph3::SSN::Full_Serial_RLC::setParameters, "R"_a, "L"_a,
           "C"_a)
      .def("connect", &CPS::EMT::Ph3::SSN::Full_Serial_RLC::connect)
      .def_property("R", createAttributeGetter<CPS::Real>("R"),
                    createAttributeSetter<CPS::Real>("R"))
      .def_property("L", createAttributeGetter<CPS::Real>("L"),
                    createAttributeSetter<CPS::Real>("L"))
      .def_property("C", createAttributeGetter<CPS::Real>("C"),
                    createAttributeSetter<CPS::Real>("C"));
}
