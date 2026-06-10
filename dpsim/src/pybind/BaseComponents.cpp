/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include <dpsim-models/Base/Base_PSS.h>
#include <dpsim-models/CSVReader.h>
#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/Signal/PSS1A.h>
#include <dpsim-models/Signal/TurbineGovernorType1.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim/Simulation.h>
#include <dpsim/pybind/BaseComponents.h>
#include <dpsim/pybind/Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addBaseComponents(py::module_ mBase) {
  py::class_<CPS::Base::Ph1::Switch, std::shared_ptr<CPS::Base::Ph1::Switch>>(
      mBase, "Switch");
  py::class_<CPS::Base::Ph3::Switch, std::shared_ptr<CPS::Base::Ph3::Switch>>(
      mBase, "SwitchPh3");

  py::class_<CPS::MNASyncGenInterface,
             std::shared_ptr<CPS::MNASyncGenInterface>>(
      mBase, "MNASyncGenInterface", py::multiple_inheritance())
      .def("set_max_iterations", &CPS::MNASyncGenInterface::setMaxIterations,
           "max_iter"_a)
      .def("set_tolerance", &CPS::MNASyncGenInterface::setTolerance,
           "tolerance"_a);

  py::class_<
      CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>,
      std::shared_ptr<CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>>,
      CPS::SimPowerComp<CPS::Complex>>(
      mBase, "ReducedOrderSynchronGeneratorComplex", py::multiple_inheritance())
      .def("set_base_parameters",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Complex>::setBaseParameters,
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a)
      .def("set_initial_values",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Complex>::setInitialValues,
           "init_complex_electrical_power"_a, "init_mechanical_power"_a,
           "init_complex_terminal_voltage"_a)
      .def("scale_inertia_constant",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Complex>::scaleInertiaConstant,
           "scaling_factor"_a)
      .def("set_model_as_norton_source",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Complex>::setModelAsNortonSource,
           "model_as_norton_source"_a)
      .def(
          "add_exciter",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex> &self,
             std::shared_ptr<CPS::Base::Exciter> exciter,
             std::shared_ptr<CPS::Base::ExciterParameters> parameters) {
            self.addExciter(exciter, parameters);
          },
          "exciter"_a, "parameters"_a)
      .def(
          "add_exciter",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex> &self,
             std::shared_ptr<CPS::Base::Exciter> exciter) {
            self.addExciter(exciter);
          },
          "exciter"_a)
      .def(
          "add_exciter",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex> &self,
             CPS::Real Ta, CPS::Real Ka, CPS::Real Te, CPS::Real Ke,
             CPS::Real Tf, CPS::Real Kf,
             CPS::Real Tr) { self.addExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr); },
          "Ta"_a, "Ka"_a, "Te"_a, "Ke"_a, "Tf"_a, "Kf"_a, "Tr"_a)
      .def("add_pss",
           py::overload_cast<std::shared_ptr<CPS::Base::PSS>,
                             std::shared_ptr<CPS::Base::PSSParameters>>(
               &CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>::addPSS),
           "pss"_a, "parameters"_a)
      .def("add_pss",
           py::overload_cast<std::shared_ptr<CPS::Base::PSS>>(
               &CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>::addPSS),
           "pss"_a)
      .def(
          "add_governor",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex> &self,
             std::shared_ptr<CPS::Signal::TurbineGovernorType1> governor) {
            self.addGovernor(governor);
          },
          "governor"_a)
      .def(
          "add_governor",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex> &self,
             CPS::Real T3, CPS::Real T4, CPS::Real T5, CPS::Real Tc,
             CPS::Real Ts, CPS::Real R, CPS::Real Pmin, CPS::Real Pmax,
             CPS::Real OmRef, CPS::Real TmRef) {
            self.addGovernor(T3, T4, T5, Tc, Ts, R, Pmin, Pmax, OmRef, TmRef);
          },
          "T3"_a, "T4"_a, "T5"_a, "Tc"_a, "Ts"_a, "R"_a, "Pmin"_a, "Pmax"_a,
          "OmRef"_a, "TmRef"_a);

  py::class_<
      CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>,
      std::shared_ptr<CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>>,
      CPS::SimPowerComp<CPS::Real>>(mBase, "ReducedOrderSynchronGeneratorReal",
                                    py::multiple_inheritance())
      .def("set_base_parameters",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Real>::setBaseParameters,
           "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a)
      .def("set_initial_values",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Real>::setInitialValues,
           "init_complex_electrical_power"_a, "init_mechanical_power"_a,
           "init_complex_terminal_voltage"_a)
      .def("scale_inertia_constant",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Real>::scaleInertiaConstant,
           "scaling_factor"_a)
      .def("set_model_as_norton_source",
           &CPS::Base::ReducedOrderSynchronGenerator<
               CPS::Real>::setModelAsNortonSource,
           "model_as_norton_source"_a)
      .def(
          "add_exciter",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Real> &self,
             std::shared_ptr<CPS::Base::Exciter> exciter,
             std::shared_ptr<CPS::Base::ExciterParameters> parameters) {
            self.addExciter(exciter, parameters);
          },
          "exciter"_a, "parameters"_a)
      .def(
          "add_exciter",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Real> &self,
             std::shared_ptr<CPS::Base::Exciter> exciter) {
            self.addExciter(exciter);
          },
          "exciter"_a)
      .def(
          "add_exciter",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Real> &self,
             CPS::Real Ta, CPS::Real Ka, CPS::Real Te, CPS::Real Ke,
             CPS::Real Tf, CPS::Real Kf,
             CPS::Real Tr) { self.addExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr); },
          "Ta"_a, "Ka"_a, "Te"_a, "Ke"_a, "Tf"_a, "Kf"_a, "Tr"_a)
      .def("add_pss",
           py::overload_cast<std::shared_ptr<CPS::Base::PSS>,
                             std::shared_ptr<CPS::Base::PSSParameters>>(
               &CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>::addPSS),
           "pss"_a, "parameters"_a)
      .def("add_pss",
           py::overload_cast<std::shared_ptr<CPS::Base::PSS>>(
               &CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>::addPSS),
           "pss"_a)
      .def(
          "add_governor",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Real> &self,
             std::shared_ptr<CPS::Signal::TurbineGovernorType1> governor) {
            self.addGovernor(governor);
          },
          "governor"_a)
      .def(
          "add_governor",
          [](CPS::Base::ReducedOrderSynchronGenerator<CPS::Real> &self,
             CPS::Real T3, CPS::Real T4, CPS::Real T5, CPS::Real Tc,
             CPS::Real Ts, CPS::Real R, CPS::Real Pmin, CPS::Real Pmax,
             CPS::Real OmRef, CPS::Real TmRef) {
            self.addGovernor(T3, T4, T5, Tc, Ts, R, Pmin, Pmax, OmRef, TmRef);
          },
          "T3"_a, "T4"_a, "T5"_a, "Tc"_a, "Ts"_a, "R"_a, "Pmin"_a, "Pmax"_a,
          "OmRef"_a, "TmRef"_a);
}
