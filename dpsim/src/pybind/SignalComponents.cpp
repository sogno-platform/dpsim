/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim-models/Base/Base_Governor.h>
#include <dpsim-models/Base/Base_PSS.h>
#include <dpsim-models/Base/Base_Turbine.h>
#include <dpsim-models/CSVReader.h>
#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/Signal/HydroTurbine.h>
#include <dpsim-models/Signal/HydroTurbineGovernor.h>
#include <dpsim-models/Signal/PSS1A.h>
#include <dpsim-models/Signal/SteamTurbine.h>
#include <dpsim-models/Signal/SteamTurbineGovernor.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim/Simulation.h>
#include <dpsim/pybind/SignalComponents.h>
#include <dpsim/pybind/Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addSignalComponents(py::module_ mSignal) {

  py::class_<CPS::TopologicalSignalComp,
             std::shared_ptr<CPS::TopologicalSignalComp>,
             CPS::IdentifiedObject>(mSignal, "TopologicalSignalComp");
  py::class_<CPS::SimSignalComp, std::shared_ptr<CPS::SimSignalComp>,
             CPS::TopologicalSignalComp>(mSignal, "SimSignalComp");

  py::class_<CPS::Base::Exciter, std::shared_ptr<CPS::Base::Exciter>>(
      mSignal, "Exciter");

  py::class_<CPS::Base::ExciterParameters,
             std::shared_ptr<CPS::Base::ExciterParameters>>(
      mSignal, "ExciterParameters");

  py::class_<CPS::Signal::ExciterDC1Parameters,
             std::shared_ptr<CPS::Signal::ExciterDC1Parameters>,
             CPS::Base::ExciterParameters>(mSignal, "ExciterDC1Parameters")
      .def(py::init<>())
      .def_readwrite("Tr", &CPS::Signal::ExciterDC1Parameters::Tr)
      .def_readwrite("Ta", &CPS::Signal::ExciterDC1Parameters::Ta)
      .def_readwrite("Tb", &CPS::Signal::ExciterDC1Parameters::Tb)
      .def_readwrite("Tc", &CPS::Signal::ExciterDC1Parameters::Tc)
      .def_readwrite("Tef", &CPS::Signal::ExciterDC1Parameters::Tef)
      .def_readwrite("Tf", &CPS::Signal::ExciterDC1Parameters::Tf)
      .def_readwrite("Ka", &CPS::Signal::ExciterDC1Parameters::Ka)
      .def_readwrite("Kef", &CPS::Signal::ExciterDC1Parameters::Kef)
      .def_readwrite("Kf", &CPS::Signal::ExciterDC1Parameters::Kf)
      .def_readwrite("Aef", &CPS::Signal::ExciterDC1Parameters::Aef)
      .def_readwrite("Bef", &CPS::Signal::ExciterDC1Parameters::Bef)
      .def_readwrite("MaxVa", &CPS::Signal::ExciterDC1Parameters::MaxVa)
      .def_readwrite("MinVa", &CPS::Signal::ExciterDC1Parameters::MinVa);

  py::class_<CPS::Signal::ExciterDC1SimpParameters,
             std::shared_ptr<CPS::Signal::ExciterDC1SimpParameters>,
             CPS::Base::ExciterParameters>(mSignal, "ExciterDC1SimpParameters")
      .def(py::init<>())
      .def_readwrite("Tr", &CPS::Signal::ExciterDC1SimpParameters::Tr)
      .def_readwrite("Ta", &CPS::Signal::ExciterDC1SimpParameters::Ta)
      .def_readwrite("Tef", &CPS::Signal::ExciterDC1SimpParameters::Tef)
      .def_readwrite("Tf", &CPS::Signal::ExciterDC1SimpParameters::Tf)
      .def_readwrite("Ka", &CPS::Signal::ExciterDC1SimpParameters::Ka)
      .def_readwrite("Kef", &CPS::Signal::ExciterDC1SimpParameters::Kef)
      .def_readwrite("Kf", &CPS::Signal::ExciterDC1SimpParameters::Kf)
      .def_readwrite("Aef", &CPS::Signal::ExciterDC1SimpParameters::Aef)
      .def_readwrite("Bef", &CPS::Signal::ExciterDC1SimpParameters::Bef)
      .def_readwrite("MaxVa", &CPS::Signal::ExciterDC1SimpParameters::MaxVa)
      .def_readwrite("MinVa", &CPS::Signal::ExciterDC1SimpParameters::MinVa);

  py::class_<CPS::Signal::ExciterST1Parameters,
             std::shared_ptr<CPS::Signal::ExciterST1Parameters>,
             CPS::Base::ExciterParameters>(mSignal, "ExciterST1Parameters")
      .def(py::init<>())
      .def_readwrite("Tr", &CPS::Signal::ExciterST1Parameters::Tr)
      .def_readwrite("Ka", &CPS::Signal::ExciterST1Parameters::Ka)
      .def_readwrite("MaxVa", &CPS::Signal::ExciterST1Parameters::MaxVa)
      .def_readwrite("MinVa", &CPS::Signal::ExciterST1Parameters::MinVa);

  py::class_<CPS::Signal::ExciterStaticParameters,
             std::shared_ptr<CPS::Signal::ExciterStaticParameters>,
             CPS::Base::ExciterParameters>(mSignal, "ExciterStaticParameters")
      .def(py::init<>())
      .def_readwrite("Tr", &CPS::Signal::ExciterStaticParameters::Tr)
      .def_readwrite("Ta", &CPS::Signal::ExciterStaticParameters::Ta)
      .def_readwrite("Tb", &CPS::Signal::ExciterStaticParameters::Tb)
      .def_readwrite("Te", &CPS::Signal::ExciterStaticParameters::Te)
      .def_readwrite("Ka", &CPS::Signal::ExciterStaticParameters::Ka)
      .def_readwrite("MaxEfd", &CPS::Signal::ExciterStaticParameters::MaxEfd)
      .def_readwrite("MinEfd", &CPS::Signal::ExciterStaticParameters::MinEfd)
      .def_readwrite("Kbc", &CPS::Signal::ExciterStaticParameters::Kbc);

  py::class_<CPS::Signal::ExciterDC1, std::shared_ptr<CPS::Signal::ExciterDC1>,
             CPS::SimSignalComp, CPS::Base::Exciter>(mSignal, "ExciterDC1",
                                                     py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def(
          "set_parameters",
          [](CPS::Signal::ExciterDC1 &self,
             std::shared_ptr<CPS::Signal::ExciterDC1Parameters> params) {
            self.setParameters(params);
          },
          "parameters"_a);

  py::class_<CPS::Signal::ExciterDC1Simp,
             std::shared_ptr<CPS::Signal::ExciterDC1Simp>, CPS::SimSignalComp,
             CPS::Base::Exciter>(mSignal, "ExciterDC1Simp",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def(
          "set_parameters",
          [](CPS::Signal::ExciterDC1Simp &self,
             std::shared_ptr<CPS::Signal::ExciterDC1SimpParameters> params) {
            self.setParameters(params);
          },
          "parameters"_a);

  py::class_<CPS::Signal::ExciterST1Simp,
             std::shared_ptr<CPS::Signal::ExciterST1Simp>, CPS::SimSignalComp,
             CPS::Base::Exciter>(mSignal, "ExciterST1Simp",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def(
          "set_parameters",
          [](CPS::Signal::ExciterST1Simp &self,
             std::shared_ptr<CPS::Signal::ExciterST1Parameters> params) {
            self.setParameters(params);
          },
          "parameters"_a);

  py::class_<CPS::Signal::ExciterStatic,
             std::shared_ptr<CPS::Signal::ExciterStatic>, CPS::SimSignalComp,
             CPS::Base::Exciter>(mSignal, "ExciterStatic",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def(
          "set_parameters",
          [](CPS::Signal::ExciterStatic &self,
             std::shared_ptr<CPS::Signal::ExciterStaticParameters> params) {
            self.setParameters(params);
          },
          "parameters"_a);

  py::class_<CPS::Signal::DecouplingLine,
             std::shared_ptr<CPS::Signal::DecouplingLine>, CPS::SimSignalComp>(
      mSignal, "DecouplingLine", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::DecouplingLine::setParameters,
           "node_1"_a, "node_2"_a, "resistance"_a, "inductance"_a,
           "capacitance"_a)
      .def("get_line_components",
           &CPS::Signal::DecouplingLine::getLineComponents);

  py::class_<CPS::Signal::DecouplingLineEMT,
             std::shared_ptr<CPS::Signal::DecouplingLineEMT>,
             CPS::SimSignalComp>(mSignal, "DecouplingLineEMT",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::DecouplingLineEMT::setParameters,
           "node_1"_a, "node_2"_a, "resistance"_a, "inductance"_a,
           "capacitance"_a)
      .def("get_line_components",
           &CPS::Signal::DecouplingLineEMT::getLineComponents);

  py::class_<CPS::Signal::DecouplingLineEMT_Ph3,
             std::shared_ptr<CPS::Signal::DecouplingLineEMT_Ph3>,
             CPS::SimSignalComp>(mSignal, "DecouplingLineEMT_Ph3",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::DecouplingLineEMT_Ph3::setParameters,
           "node_1"_a, "node_2"_a, "resistance"_a, "inductance"_a,
           "capacitance"_a)
      .def("get_line_components",
           &CPS::Signal::DecouplingLineEMT_Ph3::getLineComponents);

  py::class_<CPS::Signal::DecouplingIdealTransformer_SP_Ph1,
             std::shared_ptr<CPS::Signal::DecouplingIdealTransformer_SP_Ph1>,
             CPS::SimSignalComp>(mSignal, "DecouplingIdealTransformer_SP_Ph1",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::Signal::DecouplingIdealTransformer_SP_Ph1::setParameters,
           "node_1"_a, "node_2"_a, "delay"_a, "v_src_intf_cur"_a,
           "cur1_extrap_0"_a, "coupling_method"_a)
      .def("get_components",
           &CPS::Signal::DecouplingIdealTransformer_SP_Ph1::getComponents)
      .def("get_virtual_node",
           &CPS::Signal::DecouplingIdealTransformer_SP_Ph1::getVirtualNode);

  py::class_<CPS::Signal::DecouplingIdealTransformer_EMT_Ph1,
             std::shared_ptr<CPS::Signal::DecouplingIdealTransformer_EMT_Ph1>,
             CPS::SimSignalComp>(mSignal, "DecouplingIdealTransformer_EMT_Ph1",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::Signal::DecouplingIdealTransformer_EMT_Ph1::setParameters,
           "node_1"_a, "node_2"_a, "delay"_a, "v_src_intf_cur"_a,
           "cur1_extrap_0"_a, "coupling_method"_a)
      .def("get_components",
           &CPS::Signal::DecouplingIdealTransformer_EMT_Ph1::getComponents);

  py::class_<CPS::Signal::DecouplingIdealTransformer_EMT_Ph3,
             std::shared_ptr<CPS::Signal::DecouplingIdealTransformer_EMT_Ph3>,
             CPS::SimSignalComp>(mSignal, "DecouplingIdealTransformer_EMT_Ph3",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::Signal::DecouplingIdealTransformer_EMT_Ph3::setParameters,
           "node_1"_a, "node_2"_a, "delay"_a, "v_src_intf_cur"_a,
           "cur1_extrap_0"_a, "coupling_method"_a)
      .def("get_components",
           &CPS::Signal::DecouplingIdealTransformer_EMT_Ph3::getComponents)
      .def("get_virtual_node",
           &CPS::Signal::DecouplingIdealTransformer_EMT_Ph3::getVirtualNode);

  py::class_<CPS::Signal::DecouplingIdealTransformer_DP_Ph1,
             std::shared_ptr<CPS::Signal::DecouplingIdealTransformer_DP_Ph1>,
             CPS::SimSignalComp>(mSignal, "DecouplingIdealTransformer_DP_Ph1",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters",
           &CPS::Signal::DecouplingIdealTransformer_DP_Ph1::setParameters,
           "node_1"_a, "node_2"_a, "delay"_a, "v_src_intf_cur"_a,
           "cur1_extrap_0"_a, "coupling_method"_a)
      .def("get_components",
           &CPS::Signal::DecouplingIdealTransformer_DP_Ph1::getComponents)
      .def("get_virtual_node",
           &CPS::Signal::DecouplingIdealTransformer_DP_Ph1::getVirtualNode);

  py::class_<CPS::Base::PSSParameters,
             std::shared_ptr<CPS::Base::PSSParameters>>(mSignal,
                                                        "PSSParameters");

  py::class_<CPS::Base::PSS, std::shared_ptr<CPS::Base::PSS>>(mSignal, "PSS");

  py::class_<CPS::Signal::PSS1AParameters,
             std::shared_ptr<CPS::Signal::PSS1AParameters>,
             CPS::Base::PSSParameters>(mSignal, "PSS1AParameters",
                                       py::multiple_inheritance())
      .def(py::init<>())
      .def_readwrite("Kp", &CPS::Signal::PSS1AParameters::Kp)
      .def_readwrite("Kv", &CPS::Signal::PSS1AParameters::Kv)
      .def_readwrite("Kw", &CPS::Signal::PSS1AParameters::Kw)
      .def_readwrite("T1", &CPS::Signal::PSS1AParameters::T1)
      .def_readwrite("T2", &CPS::Signal::PSS1AParameters::T2)
      .def_readwrite("T3", &CPS::Signal::PSS1AParameters::T3)
      .def_readwrite("T4", &CPS::Signal::PSS1AParameters::T4)
      .def_readwrite("Vs_max", &CPS::Signal::PSS1AParameters::Vs_max)
      .def_readwrite("Vs_min", &CPS::Signal::PSS1AParameters::Vs_min)
      .def_readwrite("Tw", &CPS::Signal::PSS1AParameters::Tw);

  py::class_<CPS::Signal::PSS1A, std::shared_ptr<CPS::Signal::PSS1A>,
             CPS::SimSignalComp, CPS::Base::PSS>(mSignal, "PSS1A",
                                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::PSS1A::setParameters,
           "parameters"_a);

  py::class_<CPS::Signal::TurbineGovernorType1,
             std::shared_ptr<CPS::Signal::TurbineGovernorType1>,
             CPS::SimSignalComp>(mSignal, "TurbineGovernorType1",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::TurbineGovernorType1::setParameters,
           "T3"_a, "T4"_a, "T5"_a, "Tc"_a, "Ts"_a, "R"_a, "Tmin"_a, "Tmax"_a,
           "OmRef"_a)
      .def("initialize_states",
           &CPS::Signal::TurbineGovernorType1::initializeStates, "TmRef"_a);

  // Base Governor / Turbine
  py::class_<CPS::Base::GovernorParameters,
             std::shared_ptr<CPS::Base::GovernorParameters>>(
      mSignal, "GovernorParameters");
  py::class_<CPS::Base::Governor, std::shared_ptr<CPS::Base::Governor>>(
      mSignal, "Governor");

  py::class_<CPS::Base::TurbineParameters,
             std::shared_ptr<CPS::Base::TurbineParameters>>(
      mSignal, "TurbineParameters");
  py::class_<CPS::Base::Turbine, std::shared_ptr<CPS::Base::Turbine>>(
      mSignal, "Turbine");

  // SteamTurbineGovernor
  py::class_<CPS::Signal::SteamGovernorParameters,
             std::shared_ptr<CPS::Signal::SteamGovernorParameters>,
             CPS::Base::GovernorParameters>(mSignal, "SteamGovernorParameters",
                                            py::multiple_inheritance())
      .def(py::init<>())
      .def_readwrite("R", &CPS::Signal::SteamGovernorParameters::R)
      .def_readwrite("T1", &CPS::Signal::SteamGovernorParameters::T1)
      .def_readwrite("T2", &CPS::Signal::SteamGovernorParameters::T2)
      .def_readwrite("T3", &CPS::Signal::SteamGovernorParameters::T3)
      .def_readwrite("dPmax", &CPS::Signal::SteamGovernorParameters::dPmax)
      .def_readwrite("dPmin", &CPS::Signal::SteamGovernorParameters::dPmin)
      .def_readwrite("Pmax", &CPS::Signal::SteamGovernorParameters::Pmax)
      .def_readwrite("Pmin", &CPS::Signal::SteamGovernorParameters::Pmin)
      .def_readwrite("OmRef", &CPS::Signal::SteamGovernorParameters::OmRef)
      .def_readwrite("Kbc", &CPS::Signal::SteamGovernorParameters::Kbc);

  py::class_<CPS::Signal::SteamTurbineGovernor,
             std::shared_ptr<CPS::Signal::SteamTurbineGovernor>,
             CPS::SimSignalComp, CPS::Base::Governor>(
      mSignal, "SteamTurbineGovernor", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::SteamTurbineGovernor::setParameters,
           "parameters"_a)
      .def("initialize_states",
           &CPS::Signal::SteamTurbineGovernor::initializeStates, "Pref"_a);

  // SteamTurbine
  py::class_<CPS::Signal::SteamTurbineParameters,
             std::shared_ptr<CPS::Signal::SteamTurbineParameters>,
             CPS::Base::TurbineParameters>(mSignal, "SteamTurbineParameters",
                                           py::multiple_inheritance())
      .def(py::init<>())
      .def_readwrite("Fhp", &CPS::Signal::SteamTurbineParameters::Fhp)
      .def_readwrite("Fip", &CPS::Signal::SteamTurbineParameters::Fip)
      .def_readwrite("Flp", &CPS::Signal::SteamTurbineParameters::Flp)
      .def_readwrite("Tch", &CPS::Signal::SteamTurbineParameters::Tch)
      .def_readwrite("Trh", &CPS::Signal::SteamTurbineParameters::Trh)
      .def_readwrite("Tco", &CPS::Signal::SteamTurbineParameters::Tco);

  py::class_<CPS::Signal::SteamTurbine,
             std::shared_ptr<CPS::Signal::SteamTurbine>, CPS::SimSignalComp,
             CPS::Base::Turbine>(mSignal, "SteamTurbine",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::SteamTurbine::setParameters,
           "parameters"_a)
      .def("initialize_states", &CPS::Signal::SteamTurbine::initializeStates,
           "Pminit"_a);

  // HydroTurbineGovernor
  py::class_<CPS::Signal::HydroGovernorParameters,
             std::shared_ptr<CPS::Signal::HydroGovernorParameters>,
             CPS::Base::GovernorParameters>(mSignal, "HydroGovernorParameters",
                                            py::multiple_inheritance())
      .def(py::init<>())
      .def_readwrite("R", &CPS::Signal::HydroGovernorParameters::R)
      .def_readwrite("T1", &CPS::Signal::HydroGovernorParameters::T1)
      .def_readwrite("T2", &CPS::Signal::HydroGovernorParameters::T2)
      .def_readwrite("T3", &CPS::Signal::HydroGovernorParameters::T3)
      .def_readwrite("Pmax", &CPS::Signal::HydroGovernorParameters::Pmax)
      .def_readwrite("Pmin", &CPS::Signal::HydroGovernorParameters::Pmin)
      .def_readwrite("OmRef", &CPS::Signal::HydroGovernorParameters::OmRef);

  py::class_<CPS::Signal::HydroTurbineGovernor,
             std::shared_ptr<CPS::Signal::HydroTurbineGovernor>,
             CPS::SimSignalComp, CPS::Base::Governor>(
      mSignal, "HydroTurbineGovernor", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::HydroTurbineGovernor::setParameters,
           "parameters"_a)
      .def("initialize_states",
           &CPS::Signal::HydroTurbineGovernor::initializeStates, "Pref"_a);

  // HydroTurbine
  py::class_<CPS::Signal::HydroTurbineParameters,
             std::shared_ptr<CPS::Signal::HydroTurbineParameters>,
             CPS::Base::TurbineParameters>(mSignal, "HydroTurbineParameters",
                                           py::multiple_inheritance())
      .def(py::init<>())
      .def_readwrite("Tw", &CPS::Signal::HydroTurbineParameters::Tw);

  py::class_<CPS::Signal::HydroTurbine,
             std::shared_ptr<CPS::Signal::HydroTurbine>, CPS::SimSignalComp,
             CPS::Base::Turbine>(mSignal, "HydroTurbine",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::HydroTurbine::setParameters,
           "parameters"_a)
      .def("initialize_states", &CPS::Signal::HydroTurbine::initializeStates,
           "Pminit"_a);
}
