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

  py::class_<CPS::Signal::TurbineGovernorType1,
             std::shared_ptr<CPS::Signal::TurbineGovernorType1>,
             CPS::SimSignalComp>(mSignal, "TurbineGovernorType1",
                                 py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::TurbineGovernorType1::setParameters,
           "T3"_a, "T4"_a, "T5"_a, "Tc"_a, "Ts"_a, "R"_a, "Tmin"_a, "Tmax"_a,
           "OmRef"_a);
}
