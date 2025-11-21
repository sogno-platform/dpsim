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

  py::class_<CPS::Signal::Exciter, std::shared_ptr<CPS::Signal::Exciter>,
             CPS::SimSignalComp>(mSignal, "Exciter", py::multiple_inheritance())
      .def(py::init<std::string>())
      .def(py::init<std::string, CPS::Logger::Level>())
      .def("set_parameters", &CPS::Signal::Exciter::setParameters, "Ta"_a,
           "Ka"_a, "Te"_a, "Ke"_a, "Tf"_a, "Kf"_a, "Tr"_a, "max_vr"_a = 1.0,
           "min_vr"_a = -0.9);

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
