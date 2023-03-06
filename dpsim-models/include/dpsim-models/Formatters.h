/* Copyright 2023 OPAL-RT Germany GmbH
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#if __has_include(<fmt/ostream.h>)

#include <fmt/ostream.h>

namespace fmt {

template <>
struct formatter<CPS::Complex> : ostream_formatter {};

template <>
struct formatter<CPS::Vector> : ostream_formatter {};

template <>
struct formatter<CPS::VectorComp> : ostream_formatter {};

template <>
struct formatter<CPS::Matrix> : ostream_formatter {};

template <>
struct formatter<CPS::MatrixComp> : ostream_formatter {};

template <>
struct formatter<CPS::SparseMatrix> : ostream_formatter {};

template <>
struct formatter<CPS::SparseMatrixRow> : ostream_formatter {};

template <>
struct formatter<CPS::SparseMatrixComp> : ostream_formatter {};

template <>
struct formatter<Eigen::Block<CPS::Matrix>> : ostream_formatter {};

}

#endif
