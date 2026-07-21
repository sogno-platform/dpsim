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

#ifdef WITH_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

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

  py::class_<CPS::EMT::Ph1::SSN::Full_Serial_RLC,
             std::shared_ptr<CPS::EMT::Ph1::SSN::Full_Serial_RLC>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "Full_Serial_RLC",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::EMT::Ph1::SSN::Full_Serial_RLC::setParameters, "R"_a, "L"_a,
           "C"_a)
      .def("connect", &CPS::EMT::Ph1::SSN::Full_Serial_RLC::connect)
      .def_property("R", createAttributeGetter<CPS::Real>("R"),
                    createAttributeSetter<CPS::Real>("R"))
      .def_property("L", createAttributeGetter<CPS::Real>("L"),
                    createAttributeSetter<CPS::Real>("L"))
      .def_property("C", createAttributeGetter<CPS::Real>("C"),
                    createAttributeSetter<CPS::Real>("C"));

  py::class_<CPS::EMT::Ph1::SSNTypeI2T,
             std::shared_ptr<CPS::EMT::Ph1::SSNTypeI2T>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "SSNTypeI2T",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::SSNTypeI2T::setParameters, "A"_a,
           "B"_a, "C"_a, "D"_a)
      .def("manual_init", &CPS::EMT::Ph1::SSNTypeI2T::manualInit,
           "initialState"_a, "initialInput"_a, "initialOldInput"_a,
           "initCurr"_a, "initVol"_a)
      .def("connect", &CPS::EMT::Ph1::SSNTypeI2T::connect)
      .def_property("mA", createAttributeGetter<CPS::Matrix>("mA"),
                    createAttributeSetter<CPS::Matrix>("mA"))
      .def_property("mB", createAttributeGetter<CPS::Matrix>("mB"),
                    createAttributeSetter<CPS::Matrix>("mB"))
      .def_property("mC", createAttributeGetter<CPS::Matrix>("mC"),
                    createAttributeSetter<CPS::Matrix>("mC"))
      .def_property("mD", createAttributeGetter<CPS::Matrix>("mD"),
                    createAttributeSetter<CPS::Matrix>("mD"))
      .def_property("mdA", createAttributeGetter<CPS::Matrix>("mdA"),
                    createAttributeSetter<CPS::Matrix>("mdA"))
      .def_property("mdB", createAttributeGetter<CPS::Matrix>("mdB"),
                    createAttributeSetter<CPS::Matrix>("mdB"))
      .def_property("mdC", createAttributeGetter<CPS::Matrix>("mdC"),
                    createAttributeSetter<CPS::Matrix>("mdC"));

  py::class_<CPS::EMT::Ph1::SSNTypeV2T,
             std::shared_ptr<CPS::EMT::Ph1::SSNTypeV2T>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh1, "SSNTypeV2T",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph1::SSNTypeV2T::setParameters, "A"_a,
           "B"_a, "C"_a, "D"_a)
      .def("manual_init", &CPS::EMT::Ph1::SSNTypeV2T::manualInit,
           "initialState"_a, "initialInput"_a, "initialOldInput"_a,
           "initCurr"_a, "initVol"_a)
      .def("connect", &CPS::EMT::Ph1::SSNTypeV2T::connect)
      .def_property("mA", createAttributeGetter<CPS::Matrix>("mA"),
                    createAttributeSetter<CPS::Matrix>("mA"))
      .def_property("mB", createAttributeGetter<CPS::Matrix>("mB"),
                    createAttributeSetter<CPS::Matrix>("mB"))
      .def_property("mC", createAttributeGetter<CPS::Matrix>("mC"),
                    createAttributeSetter<CPS::Matrix>("mC"))
      .def_property("mD", createAttributeGetter<CPS::Matrix>("mD"),
                    createAttributeSetter<CPS::Matrix>("mD"))
      .def_property("mdA", createAttributeGetter<CPS::Matrix>("mdA"),
                    createAttributeSetter<CPS::Matrix>("mdA"))
      .def_property("mdB", createAttributeGetter<CPS::Matrix>("mdB"),
                    createAttributeSetter<CPS::Matrix>("mdB"))
      .def_property("mdC", createAttributeGetter<CPS::Matrix>("mdC"),
                    createAttributeSetter<CPS::Matrix>("mdC"));
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

  py::class_<CPS::EMT::Ph3::ControlledVoltageSource,
             std::shared_ptr<CPS::EMT::Ph3::ControlledVoltageSource>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "ControlledVoltageSource",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           py::overload_cast<CPS::Matrix>(
               &CPS::EMT::Ph3::ControlledVoltageSource::setParameters),
           "V_ref"_a)
      .def("connect", &CPS::EMT::Ph3::ControlledVoltageSource::connect)
      .def_property("V_ref", createAttributeGetter<CPS::MatrixComp>("V_ref"),
                    createAttributeSetter<CPS::MatrixComp>("V_ref"));

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
      .def("connect", &CPS::EMT::Ph3::CurrentSource::connect)
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
           "active_power"_a, "reactive_power"_a, "volt"_a,
           // cppcheck-suppress assignBoolToPointer
           "reactance_in_series"_a = false)
      .def("connect", &CPS::EMT::Ph3::RXLoad::connect);

  py::class_<CPS::EMT::Ph3::Shunt, std::shared_ptr<CPS::EMT::Ph3::Shunt>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "Shunt",
                                           py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters",
           static_cast<void (CPS::EMT::Ph3::Shunt::*)(CPS::Real, CPS::Real)>(
               &CPS::EMT::Ph3::Shunt::setParameters),
           "G"_a, "B"_a)
      .def("connect", &CPS::EMT::Ph3::Shunt::connect);

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
#ifdef WITH_JSON
      .def("apply_parameters_from_json",
           [](std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQTrapez> syngen,
              const CPS::String json) {
             DPsim::Utils::applySynchronousGeneratorParametersFromJson(
                 json::parse(json), syngen);
           })
#endif
      .def("set_initial_values",
           &CPS::EMT::Ph3::SynchronGeneratorDQTrapez::setInitialValues,
           "init_active_power"_a, "init_reactive_power"_a,
           "init_terminal_volt"_a, "init_volt_angle"_a, "init_mech_power"_a);

  py::class_<CPS::EMT::Ph3::VSIVoltageControlVCO,
             std::shared_ptr<CPS::EMT::Ph3::VSIVoltageControlVCO>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "VSIVoltageControlVCO",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def(py::init<std::string, std::string, CPS::Logger::Level, CPS::Bool>(),
           "uid"_a, "name"_a, "log_level"_a = CPS::Logger::Level::off,
           py::arg("with_trafo") = false)
      .def("set_parameters",
           &CPS::EMT::Ph3::VSIVoltageControlVCO::setParameters, "sys_omega"_a,
           "vd_ref"_a, "vq_ref"_a)
      .def("set_controller_parameters",
           &CPS::EMT::Ph3::VSIVoltageControlVCO::setControllerParameters,
           "kp_voltage_ctrl"_a, "ki_voltage_ctrl"_a, "kp_curr_ctrl"_a,
           "ki_curr_ctrl"_a, "omega_nominal"_a)
      .def("set_transformer_parameters",
           &CPS::EMT::Ph3::VSIVoltageControlVCO::setTransformerParameters,
           "nom_voltage_end1"_a, "nom_voltage_end2"_a, "rated_power"_a,
           "ratio_abs"_a, "ratio_phase"_a, "resistance"_a, "inductance"_a,
           "omega"_a)
      .def("set_filter_parameters",
           &CPS::EMT::Ph3::VSIVoltageControlVCO::setFilterParameters, "lf"_a,
           "cf"_a, "rf"_a, "rc"_a)
      .def("set_initial_state_values",
           &CPS::EMT::Ph3::VSIVoltageControlVCO::setInitialStateValues,
           "phi_d_init"_a, "phi_q_init"_a, "gamma_d_init"_a, "gamma_q_init"_a)
      .def("with_control", &CPS::EMT::Ph3::VSIVoltageControlVCO::withControl,
           "control_on"_a)
      .def("connect", &CPS::EMT::Ph3::VSIVoltageControlVCO::connect);

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
#ifdef WITH_JSON
      .def("apply_parameters_from_json",
           [](std::shared_ptr<CPS::EMT::Ph3::SynchronGeneratorDQODE> syngen,
              const CPS::String json) {
             DPsim::Utils::applySynchronousGeneratorParametersFromJson(
                 json::parse(json), syngen);
           })
#endif
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
      .def_property("R", createAttributeGetter<CPS::Matrix>("R"),
                    createAttributeSetter<CPS::Matrix>("R"))
      .def_property("L", createAttributeGetter<CPS::Matrix>("L"),
                    createAttributeSetter<CPS::Matrix>("L"))
      .def_property("C", createAttributeGetter<CPS::Matrix>("C"),
                    createAttributeSetter<CPS::Matrix>("C"));

  py::class_<CPS::EMT::Ph3::GenericTwoTerminalVTypeSSN,
             std::shared_ptr<CPS::EMT::Ph3::GenericTwoTerminalVTypeSSN>,
             CPS::SimPowerComp<CPS::Real>>(
      mEMTPh3, "GenericTwoTerminalVTypeSSN", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::EMT::Ph3::GenericTwoTerminalVTypeSSN::setParameters, "A"_a,
           "B"_a, "C"_a, "D"_a)
      .def("connect", &CPS::EMT::Ph3::GenericTwoTerminalVTypeSSN::connect)
      .def_property_readonly("x", createAttributeGetter<CPS::Matrix>("x"));

  py::class_<CPS::EMT::Ph3::GenericTwoTerminalITypeSSN,
             std::shared_ptr<CPS::EMT::Ph3::GenericTwoTerminalITypeSSN>,
             CPS::SimPowerComp<CPS::Real>>(
      mEMTPh3, "GenericTwoTerminalITypeSSN", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::EMT::Ph3::GenericTwoTerminalITypeSSN::setParameters, "A"_a,
           "B"_a, "C"_a, "D"_a)
      .def("connect", &CPS::EMT::Ph3::GenericTwoTerminalITypeSSN::connect)
      .def_property_readonly("x", createAttributeGetter<CPS::Matrix>("x"));

  py::class_<CPS::EMT::Ph3::PiecewiseLinearInductor,
             std::shared_ptr<CPS::EMT::Ph3::PiecewiseLinearInductor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "PiecewiseLinearInductor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::EMT::Ph3::PiecewiseLinearInductor::setParameters,
           "flux_breakpoints"_a, "current_breakpoints"_a)
      .def("connect", &CPS::EMT::Ph3::PiecewiseLinearInductor::connect)
      .def_property_readonly("x", createAttributeGetter<CPS::Matrix>("x"));

  py::class_<CPS::EMT::Ph3::AvVoltSourceInverterStateSpace,
             std::shared_ptr<CPS::EMT::Ph3::AvVoltSourceInverterStateSpace>,
             CPS::SimPowerComp<CPS::Real>>(
      mEMTPh3, "AvVoltSourceInverterStateSpace", py::multiple_inheritance())
      .def(py::init<std::string, CPS::Logger::Level>(), "name"_a,
           "loglevel"_a = CPS::Logger::Level::off)
      .def(py::init<std::string, std::string, CPS::Logger::Level>(), "uid"_a,
           "name"_a, "loglevel"_a = CPS::Logger::Level::off)
      .def("set_parameters",
           &CPS::EMT::Ph3::AvVoltSourceInverterStateSpace::setParameters,
           "Lf"_a, "Cf"_a, "Rf"_a, "Rc"_a, "omega_n"_a, "Kp_pll"_a, "Ki_pll"_a,
           "omega_cutoff"_a, "p_ref"_a, "q_ref"_a, "Kp_power_ctrl"_a,
           "Ki_power_ctrl"_a, "Kp_curr_ctrl"_a, "Ki_curr_ctrl"_a)
      .def("connect", &CPS::EMT::Ph3::AvVoltSourceInverterStateSpace::connect)
      .def_property_readonly("x", createAttributeGetter<CPS::Matrix>("x"))
      .def_property_readonly("vc_d", createAttributeGetter<CPS::Real>("vc_d"))
      .def_property_readonly("vc_q", createAttributeGetter<CPS::Real>("vc_q"))
      .def_property_readonly("irc_d", createAttributeGetter<CPS::Real>("irc_d"))
      .def_property_readonly("irc_q", createAttributeGetter<CPS::Real>("irc_q"))
      .def_property_readonly("p_inst",
                             createAttributeGetter<CPS::Real>("p_inst"))
      .def_property_readonly("q_inst",
                             createAttributeGetter<CPS::Real>("q_inst"))
      .def_property_readonly("omega_pll",
                             createAttributeGetter<CPS::Real>("omega_pll"));

  py::class_<CPS::EMT::Ph3::SSN_GFM, std::shared_ptr<CPS::EMT::Ph3::SSN_GFM>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "SSN_GFM",
                                           py::multiple_inheritance())
      .def(py::init<std::string, std::string, CPS::Logger::Level>(), "uid"_a,
           "name"_a, "log_level"_a = CPS::Logger::Level::off)
      .def("set_parameters", &CPS::EMT::Ph3::SSN_GFM::setParameters, "lf"_a,
           "cf"_a, "rf"_a, "rc"_a, "nominal_voltage"_a, "omega_nominal"_a,
           "p_ref"_a, "q_ref"_a, "virtual_inertia"_a, "damping_coefficient"_a,
           "voltage_droop_gain"_a, "reactive_integral_gain"_a, "kp_voltage"_a,
           "ki_voltage"_a, "kp_current"_a, "ki_current"_a,
           "active_damping_gain"_a, "power_filter_cutoff"_a,
           "delay_bandwidth"_a)
      .def("set_numerical_linearization_parameters",
           &CPS::EMT::Ph3::SSN_GFM::setNumericalLinearizationParameters,
           "relative_step"_a = 1e-6, "absolute_step"_a = 1e-8)
      .def("connect", &CPS::EMT::Ph3::SSN_GFM::connect)
      .def("get_state_names", &CPS::EMT::Ph3::SSN_GFM::getLocalStateNames)
      .def("get_state", &CPS::EMT::Ph3::SSN_GFM::getState)
      .def("get_state_derivative", &CPS::EMT::Ph3::SSN_GFM::getStateDerivative)
      .def("get_interface_voltage",
           &CPS::EMT::Ph3::SSN_GFM::getInterfaceVoltage)
      .def("get_interface_current",
           &CPS::EMT::Ph3::SSN_GFM::getInterfaceCurrent)
      .def_property_readonly("p_inst",
                             createAttributeGetter<CPS::Real>("p_inst"))
      .def_property_readonly("q_inst",
                             createAttributeGetter<CPS::Real>("q_inst"))
      .def_property_readonly("omega_gfm",
                             createAttributeGetter<CPS::Real>("omega_gfm"))
      .def_property_readonly("theta_gfm",
                             createAttributeGetter<CPS::Real>("theta_gfm"))
      .def_property_readonly(
          "voltage_magnitude_gfm",
          createAttributeGetter<CPS::Real>("voltage_magnitude_gfm"))
      .def_property_readonly("vc_d", createAttributeGetter<CPS::Real>("vc_d"))
      .def_property_readonly("vc_q", createAttributeGetter<CPS::Real>("vc_q"))
      .def_property_readonly("i_grid_d",
                             createAttributeGetter<CPS::Real>("i_grid_d"))
      .def_property_readonly("i_grid_q",
                             createAttributeGetter<CPS::Real>("i_grid_q"))
      .def_property_readonly("if_d", createAttributeGetter<CPS::Real>("if_d"))
      .def_property_readonly("if_q", createAttributeGetter<CPS::Real>("if_q"))
      .def_property_readonly("v_ref_d",
                             createAttributeGetter<CPS::Real>("v_ref_d"))
      .def_property_readonly("v_ref_q",
                             createAttributeGetter<CPS::Real>("v_ref_q"));

  py::class_<CPS::EMT::Ph3::SSN_SynchronousGenerator,
             std::shared_ptr<CPS::EMT::Ph3::SSN_SynchronousGenerator>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "SSN_SynchronousGenerator",
                                           py::multiple_inheritance())
      .def(py::init<std::string, std::string, CPS::Logger::Level>(), "uid"_a,
           "name"_a, "log_level"_a = CPS::Logger::Level::off)
      .def("set_parameters",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::setParameters,
           "nominal_frequency"_a, "pole_pairs"_a, "stator_resistance"_a,
           "field_resistance"_a, "damper_resistance_d"_a,
           "damper_resistance_q1"_a, "damper_resistance_q2"_a, "ld"_a, "lq"_a,
           "lmd"_a, "lmq"_a, "field_inductance"_a, "damper_inductance_d"_a,
           "damper_inductance_q1"_a, "damper_inductance_q2"_a,
           "rotor_inertia"_a, "mechanical_damping"_a, "field_voltage"_a,
           "mechanical_torque"_a, "initial_electrical_angle"_a = 0.0,
           "auto_initialize_mechanical_torque"_a = true)
      .def("set_field_voltage",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::setFieldVoltage,
           "field_voltage"_a)
      .def("set_mechanical_torque",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::setMechanicalTorque,
           "mechanical_torque"_a)
      .def("set_numerical_linearization_parameters",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::
               setNumericalLinearizationParameters,
           "relative_step"_a = 1e-6, "absolute_step"_a = 1e-8)
      .def("connect", &CPS::EMT::Ph3::SSN_SynchronousGenerator::connect)
      .def("get_state_names",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::getStateNames)
      .def("get_state", &CPS::EMT::Ph3::SSN_SynchronousGenerator::getState)
      .def("get_state_derivative",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::getStateDerivative)
      .def("get_interface_voltage",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::getInterfaceVoltage)
      .def("get_interface_current",
           &CPS::EMT::Ph3::SSN_SynchronousGenerator::getInterfaceCurrent)
      .def_property_readonly(
          "electrical_power",
          createAttributeGetter<CPS::Real>("electrical_power"))
      .def_property_readonly(
          "electrical_torque",
          createAttributeGetter<CPS::Real>("electrical_torque"))
      .def_property_readonly(
          "mechanical_speed",
          createAttributeGetter<CPS::Real>("mechanical_speed"))
      .def_property_readonly(
          "electrical_angle",
          createAttributeGetter<CPS::Real>("electrical_angle"))
      .def_property_readonly(
          "stator_current_d",
          createAttributeGetter<CPS::Real>("stator_current_d"))
      .def_property_readonly(
          "stator_current_q",
          createAttributeGetter<CPS::Real>("stator_current_q"))
      .def_property_readonly(
          "stator_voltage_d",
          createAttributeGetter<CPS::Real>("stator_voltage_d"))
      .def_property_readonly(
          "stator_voltage_q",
          createAttributeGetter<CPS::Real>("stator_voltage_q"))
      .def_property_readonly("field_current",
                             createAttributeGetter<CPS::Real>("field_current"));

  py::class_<CPS::EMT::Ph3::SSN::Capacitor,
             std::shared_ptr<CPS::EMT::Ph3::SSN::Capacitor>,
             CPS::SimPowerComp<CPS::Real>>(mEMTPh3, "SSN_Capacitor",
                                           py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::EMT::Ph3::SSN::Capacitor::setParameters,
           "C"_a)
      .def("connect", &CPS::EMT::Ph3::SSN::Capacitor::connect)
      .def_property("C", createAttributeGetter<CPS::Matrix>("C"),
                    createAttributeSetter<CPS::Matrix>("C"));

  py::class_<CPS::EMT::Ph3::GenericFourTerminalVTypeSSN,
             std::shared_ptr<CPS::EMT::Ph3::GenericFourTerminalVTypeSSN>,
             CPS::SimPowerComp<CPS::Real>>(
      mEMTPh3, "GenericFourTerminalVTypeSSN", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::EMT::Ph3::GenericFourTerminalVTypeSSN::setParameters, "A"_a,
           "B"_a, "C"_a, "D"_a)
      .def("connect", &CPS::EMT::Ph3::GenericFourTerminalVTypeSSN::connect)
      .def_property_readonly("x", createAttributeGetter<CPS::Matrix>("x"));
}
