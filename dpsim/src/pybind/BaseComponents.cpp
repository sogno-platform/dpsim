/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/pybind/BaseComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <dpsim-models/IdentifiedObject.h>
#include <DPsim.h>
#include <dpsim-models/CSVReader.h>
#include <dpsim/pybind/Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addBaseComponents(py::module_ mBase) {
    py::class_<CPS::Base::Ph1::Switch, std::shared_ptr<CPS::Base::Ph1::Switch>>(mBase, "Switch");
    py::class_<CPS::Base::Ph3::Switch, std::shared_ptr<CPS::Base::Ph3::Switch>>(mBase, "SwitchPh3");

    py::class_<CPS::MNASyncGenInterface, std::shared_ptr<CPS::MNASyncGenInterface>>(mBase, "MNASyncGenInterface", py::multiple_inheritance())
        .def("set_max_iterations", &CPS::MNASyncGenInterface::setMaxIterations, "max_iter"_a)
		.def("set_tolerance", &CPS::MNASyncGenInterface::setTolerance, "tolerance"_a);

    py::class_<CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>, std::shared_ptr<CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>>, CPS::SimPowerComp<CPS::Complex>>(mBase, "ReducedOrderSynchronGeneratorComplex", py::multiple_inheritance())
        .def("set_base_parameters", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>::setBaseParameters, "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a)
		.def("set_initial_values", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>::setInitialValues, "init_complex_electrical_power"_a, "init_mechanical_power"_a, "init_complex_terminal_voltage"_a)
		.def("scale_inertia_constant", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>::scaleInertiaConstant, "scaling_factor"_a)
		.def("set_model_as_norton_source", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>::setModelAsNortonSource, "model_as_norton_source"_a);

    py::class_<CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>, std::shared_ptr<CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>>, CPS::SimPowerComp<CPS::Real>>(mBase, "ReducedOrderSynchronGeneratorReal", py::multiple_inheritance())
        .def("set_base_parameters", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>::setBaseParameters, "nom_power"_a, "nom_voltage"_a, "nom_frequency"_a)
		.def("set_initial_values", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>::setInitialValues, "init_complex_electrical_power"_a, "init_mechanical_power"_a, "init_complex_terminal_voltage"_a)
		.def("scale_inertia_constant", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>::scaleInertiaConstant, "scaling_factor"_a)
		.def("set_model_as_norton_source", &CPS::Base::ReducedOrderSynchronGenerator<CPS::Real>::setModelAsNortonSource, "model_as_norton_source"_a);

    py::class_<CPS::Base::ExciterParameters, std::shared_ptr<CPS::Base::ExciterParameters>>(mBase, "ExciterParameters")
        .def(py::init());
}
