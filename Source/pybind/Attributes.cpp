/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <BaseComponents.h>
#include <dpsim/Simulation.h>
#include <dpsim/RealTimeSimulation.h>
#include <cps/IdentifiedObject.h>
#include <cps/CIM/Reader.h>
#include <DPsim.h>
#include <cps/CSVReader.h>
#include <Utils.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addAttributes(py::module_ m) {

    ///FIXME: Improve error reporting

    py::class_<CPS::AttributeBase, std::shared_ptr<CPS::AttributeBase>>(m, "Attribute")
		.def("derive_real", [](CPS::AttributeBase::Ptr &attr) {
			if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(attr)) {
				return attrComp->deriveReal();
			} else {
				throw CPS::InvalidAttributeException();
			}
		})
		.def("derive_imag", [](CPS::AttributeBase::Ptr &attr) {
			if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(attr)) {
				return attrComp->deriveImag();
			} else {
				throw CPS::InvalidAttributeException();
			}
		})
		.def("derive_mag", [](CPS::AttributeBase::Ptr &attr) {
			if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(attr)) {
				return attrComp->deriveMag();
			} else {
				throw CPS::InvalidAttributeException();
			}
		})
		.def("derive_phase", [](CPS::AttributeBase::Ptr &attr) {
			if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(attr)) {
				return attrComp->derivePhase();
			} else {
				throw CPS::InvalidAttributeException();
			}
		})
        .def("derive_scaled_comp", [](CPS::AttributeBase::Ptr &attr, CPS::Complex scale) {
			if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<CPS::Complex>>(attr)) {
				return attrComp->deriveScaled(scale);
            } else {
				throw CPS::InvalidAttributeException();
			}
		})
        .def("derive_scaled_real", [](CPS::AttributeBase::Ptr &attr, CPS::Real scale) {
			if (auto attrReal = std::dynamic_pointer_cast<CPS::Attribute<CPS::Real>>(attr)) {
            
				return attrReal->deriveScaled(scale);
			} else {
				throw CPS::InvalidAttributeException();
			}
		})
        .def("derive_coeff_comp", [](CPS::AttributeBase::Ptr &attr, CPS::MatrixVar<CPS::Complex>::Index row, CPS::MatrixVar<CPS::Complex>::Index column) {
			if (auto attrMatComp = std::dynamic_pointer_cast<CPS::Attribute<CPS::MatrixVar<CPS::Complex>>>(attr)) {
				return attrMatComp->deriveCoeff<CPS::Complex>(row, column);
            } else {
				throw CPS::InvalidAttributeException();
			}
		})
        .def("derive_coeff_real", [](CPS::AttributeBase::Ptr &attr, CPS::MatrixVar<CPS::Real>::Index row, CPS::MatrixVar<CPS::Real>::Index column) {
			if (auto attrMatReal = std::dynamic_pointer_cast<CPS::Attribute<CPS::MatrixVar<CPS::Real>>>(attr)) {
				return attrMatReal->deriveCoeff<CPS::Real>(row, column);
            } else {
				throw CPS::InvalidAttributeException();
			}
		});
}