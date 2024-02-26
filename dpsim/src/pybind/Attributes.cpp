/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
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
#include <dpsim/pybind/BaseComponents.h>
#include <dpsim/pybind/Utils.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, CPS::AttributePointer<T>);

namespace py = pybind11;
using namespace pybind11::literals;

void addAttributes(py::module_ m) {

  py::class_<CPS::AttributeBase, CPS::AttributePointer<CPS::AttributeBase>>(
      m, "Attribute")
      .def("__str__", &CPS::AttributeBase::toString)
      .def("__repr__", &CPS::AttributeBase::toString);

  // Class bindings for the most common attribute types. Allows for the usage of the `get` and `set` methods in Python.

  py::class_<CPS::Attribute<CPS::Real>,
             CPS::AttributePointer<CPS::Attribute<CPS::Real>>,
             CPS::AttributeBase>(m, "AttributeReal")
      .def("get", &CPS::Attribute<CPS::Real>::get)
      .def("set", &CPS::Attribute<CPS::Real>::set)
      .def("derive_scaled", &CPS::Attribute<CPS::Real>::deriveScaled);

  py::class_<CPS::AttributeStatic<CPS::Real>,
             CPS::AttributePointer<CPS::AttributeStatic<CPS::Real>>,
             CPS::Attribute<CPS::Real>>(m, "AttributeRealStat");
  py::class_<CPS::AttributeDynamic<CPS::Real>,
             CPS::AttributePointer<CPS::AttributeDynamic<CPS::Real>>,
             CPS::Attribute<CPS::Real>>(m, "AttributeRealDyn")
      .def("set_reference", &CPS::AttributeDynamic<CPS::Real>::setReference);

  py::class_<CPS::Attribute<CPS::Complex>,
             CPS::AttributePointer<CPS::Attribute<CPS::Complex>>,
             CPS::AttributeBase>(m, "AttributeComplex")
      .def("get", &CPS::Attribute<CPS::Complex>::get)
      .def("set", &CPS::Attribute<CPS::Complex>::set)
      .def("derive_real", &CPS::Attribute<CPS::Complex>::deriveReal)
      .def("derive_imag", &CPS::Attribute<CPS::Complex>::deriveImag)
      .def("derive_mag", &CPS::Attribute<CPS::Complex>::deriveMag)
      .def("derive_phase", &CPS::Attribute<CPS::Complex>::derivePhase)
      .def("derive_scaled", &CPS::Attribute<CPS::Complex>::deriveScaled);

  py::class_<CPS::AttributeStatic<CPS::Complex>,
             CPS::AttributePointer<CPS::AttributeStatic<CPS::Complex>>,
             CPS::Attribute<CPS::Complex>>(m, "AttributeComplexStat");
  py::class_<CPS::AttributeDynamic<CPS::Complex>,
             CPS::AttributePointer<CPS::AttributeDynamic<CPS::Complex>>,
             CPS::Attribute<CPS::Complex>>(m, "AttributeComplexDyn")
      .def("set_reference", &CPS::AttributeDynamic<CPS::Complex>::setReference);

  py::class_<CPS::Attribute<CPS::Matrix>,
             CPS::AttributePointer<CPS::Attribute<CPS::Matrix>>,
             CPS::AttributeBase>(m, "AttributeMatrix")
      .def("get", &CPS::Attribute<CPS::Matrix>::get)
      .def("set", &CPS::Attribute<CPS::Matrix>::set)
      .def("derive_coeff",
           &CPS::Attribute<CPS::Matrix>::deriveCoeff<CPS::Real>);

  py::class_<CPS::AttributeStatic<CPS::Matrix>,
             CPS::AttributePointer<CPS::AttributeStatic<CPS::Matrix>>,
             CPS::Attribute<CPS::Matrix>>(m, "AttributeMatrixStat");
  py::class_<CPS::AttributeDynamic<CPS::Matrix>,
             CPS::AttributePointer<CPS::AttributeDynamic<CPS::Matrix>>,
             CPS::Attribute<CPS::Matrix>>(m, "AttributeMatrixDyn")
      .def("set_reference", &CPS::AttributeDynamic<CPS::Matrix>::setReference);

  py::class_<CPS::Attribute<CPS::MatrixComp>,
             CPS::AttributePointer<CPS::Attribute<CPS::MatrixComp>>,
             CPS::AttributeBase>(m, "AttributeMatrixComp")
      .def("get", &CPS::Attribute<CPS::MatrixComp>::get)
      .def("set", &CPS::Attribute<CPS::MatrixComp>::set)
      .def("derive_coeff",
           &CPS::Attribute<CPS::MatrixComp>::deriveCoeff<CPS::Complex>);

  py::class_<CPS::AttributeStatic<CPS::MatrixComp>,
             CPS::AttributePointer<CPS::AttributeStatic<CPS::MatrixComp>>,
             CPS::Attribute<CPS::MatrixComp>>(m, "AttributeMatrixCompStat");
  py::class_<CPS::AttributeDynamic<CPS::MatrixComp>,
             CPS::AttributePointer<CPS::AttributeDynamic<CPS::MatrixComp>>,
             CPS::Attribute<CPS::MatrixComp>>(m, "AttributeMatrixCompDyn")
      .def("set_reference",
           &CPS::AttributeDynamic<CPS::MatrixComp>::setReference);

  py::class_<CPS::Attribute<CPS::String>,
             CPS::AttributePointer<CPS::Attribute<CPS::String>>,
             CPS::AttributeBase>(m, "AttributeString")
      .def("get", &CPS::Attribute<CPS::String>::get)
      .def("set", &CPS::Attribute<CPS::String>::set);

  py::class_<CPS::AttributeStatic<CPS::String>,
             CPS::AttributePointer<CPS::AttributeStatic<CPS::String>>,
             CPS::Attribute<CPS::String>>(m, "AttributeStringStat");
  py::class_<CPS::AttributeDynamic<CPS::String>,
             CPS::AttributePointer<CPS::AttributeDynamic<CPS::String>>,
             CPS::Attribute<CPS::String>>(m, "AttributeStringDyn")
      .def("set_reference", &CPS::AttributeDynamic<CPS::String>::setReference);
}
