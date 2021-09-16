/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

void addEMTComponents(py::module_ mEMT);
void addEMTPh1Components(py::module_ mEMTPh1);
void addEMTPh3Components(py::module_ mEMTPh3);
