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

  auto attribute =
      py::class_<CPS::AttributeBase, CPS::AttributePointer<CPS::AttributeBase>>(
          m, "Attribute")
          .def("__str__", &CPS::AttributeBase::toString)
          .def("__repr__", &CPS::AttributeBase::toString);

  // Class bindings for the most common attribute types. Allows for the usage of the `get` and `set` methods in Python.

  auto attribute_real =
      py::class_<CPS::Attribute<CPS::Real>,
                 CPS::AttributePointer<CPS::Attribute<CPS::Real>>,
                 CPS::AttributeBase>(m, "AttributeReal")
          .def("get", &CPS::Attribute<CPS::Real>::get)
          .def("set", &CPS::Attribute<CPS::Real>::set)
          .def("derive_scaled",
               py::overload_cast<CPS::Real>(
                   &CPS::Attribute<CPS::Real>::template deriveScaled<
                       CPS::Real>));

  auto attribute_real_static =
      py::class_<CPS::AttributeStatic<CPS::Real>,
                 CPS::AttributePointer<CPS::AttributeStatic<CPS::Real>>,
                 CPS::Attribute<CPS::Real>>(m, "AttributeRealStat");
  auto attribute_real_dynamic =
      py::class_<CPS::AttributeDynamic<CPS::Real>,
                 CPS::AttributePointer<CPS::AttributeDynamic<CPS::Real>>,
                 CPS::Attribute<CPS::Real>>(m, "AttributeRealDyn")
          .def("set_reference",
               &CPS::AttributeDynamic<CPS::Real>::setReference);

  auto attribute_complex =
      py::class_<CPS::Attribute<CPS::Complex>,
                 CPS::AttributePointer<CPS::Attribute<CPS::Complex>>,
                 CPS::AttributeBase>(m, "AttributeComplex")
          .def("get", &CPS::Attribute<CPS::Complex>::get)
          .def("set", &CPS::Attribute<CPS::Complex>::set)
          .def("derive_real",
               py::overload_cast<>(
                   &CPS::Attribute<CPS::Complex>::template deriveReal<>))
          .def("derive_imag",
               py::overload_cast<>(
                   &CPS::Attribute<CPS::Complex>::template deriveImag<>))
          .def("derive_mag",
               py::overload_cast<>(
                   &CPS::Attribute<CPS::Complex>::template deriveMag<>))
          .def("derive_phase",
               py::overload_cast<>(
                   &CPS::Attribute<CPS::Complex>::template derivePhase<>))
          .def("derive_scaled",
               py::overload_cast<CPS::Complex>(
                   &CPS::Attribute<CPS::Complex>::template deriveScaled<
                       CPS::Complex>));

  auto attribute_complex_static =
      py::class_<CPS::AttributeStatic<CPS::Complex>,
                 CPS::AttributePointer<CPS::AttributeStatic<CPS::Complex>>,
                 CPS::Attribute<CPS::Complex>>(m, "AttributeComplexStat");
  auto attribute_complex_dynamic =
      py::class_<CPS::AttributeDynamic<CPS::Complex>,
                 CPS::AttributePointer<CPS::AttributeDynamic<CPS::Complex>>,
                 CPS::Attribute<CPS::Complex>>(m, "AttributeComplexDyn")
          .def("set_reference",
               &CPS::AttributeDynamic<CPS::Complex>::setReference);

  auto attribute_matrix =
      py::class_<CPS::Attribute<CPS::Matrix>,
                 CPS::AttributePointer<CPS::Attribute<CPS::Matrix>>,
                 CPS::AttributeBase>(m, "AttributeMatrix")
          .def("get", &CPS::Attribute<CPS::Matrix>::get)
          .def("set", &CPS::Attribute<CPS::Matrix>::set)
          .def("derive_coeff",
               &CPS::Attribute<CPS::Matrix>::deriveCoeff<CPS::Real>);

  auto attribute_matrix_static =
      py::class_<CPS::AttributeStatic<CPS::Matrix>,
                 CPS::AttributePointer<CPS::AttributeStatic<CPS::Matrix>>,
                 CPS::Attribute<CPS::Matrix>>(m, "AttributeMatrixStat");
  auto attribute_matrix_dynamic =
      py::class_<CPS::AttributeDynamic<CPS::Matrix>,
                 CPS::AttributePointer<CPS::AttributeDynamic<CPS::Matrix>>,
                 CPS::Attribute<CPS::Matrix>>(m, "AttributeMatrixDyn")
          .def("set_reference",
               &CPS::AttributeDynamic<CPS::Matrix>::setReference);

  auto attribute_matrix_complex =
      py::class_<CPS::Attribute<CPS::MatrixComp>,
                 CPS::AttributePointer<CPS::Attribute<CPS::MatrixComp>>,
                 CPS::AttributeBase>(m, "AttributeMatrixComp")
          .def("get", &CPS::Attribute<CPS::MatrixComp>::get)
          .def("set", &CPS::Attribute<CPS::MatrixComp>::set)
          .def("derive_coeff",
               &CPS::Attribute<CPS::MatrixComp>::deriveCoeff<CPS::Complex>);

  auto attribute_matrix_complex_static =
      py::class_<CPS::AttributeStatic<CPS::MatrixComp>,
                 CPS::AttributePointer<CPS::AttributeStatic<CPS::MatrixComp>>,
                 CPS::Attribute<CPS::MatrixComp>>(m, "AttributeMatrixCompStat");
  auto attribute_matrix_complex_dynamic =
      py::class_<CPS::AttributeDynamic<CPS::MatrixComp>,
                 CPS::AttributePointer<CPS::AttributeDynamic<CPS::MatrixComp>>,
                 CPS::Attribute<CPS::MatrixComp>>(m, "AttributeMatrixCompDyn")
          .def("set_reference",
               &CPS::AttributeDynamic<CPS::MatrixComp>::setReference);

  auto attribute_string =
      py::class_<CPS::Attribute<CPS::String>,
                 CPS::AttributePointer<CPS::Attribute<CPS::String>>,
                 CPS::AttributeBase>(m, "AttributeString")
          .def("get", &CPS::Attribute<CPS::String>::get)
          .def("set", &CPS::Attribute<CPS::String>::set);

  auto attribute_string_static =
      py::class_<CPS::AttributeStatic<CPS::String>,
                 CPS::AttributePointer<CPS::AttributeStatic<CPS::String>>,
                 CPS::Attribute<CPS::String>>(m, "AttributeStringStat");
  auto attribute_string_dynamic =
      py::class_<CPS::AttributeDynamic<CPS::String>,
                 CPS::AttributePointer<CPS::AttributeDynamic<CPS::String>>,
                 CPS::Attribute<CPS::String>>(m, "AttributeStringDyn")
          .def("set_reference",
               &CPS::AttributeDynamic<CPS::String>::setReference);
}
