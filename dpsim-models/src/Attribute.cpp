/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Config.h>

using namespace CPS;

std::ostream &operator<<(std::ostream &output, AttributeBase &attr) {
  output << attr.toString();
  return output;
}

/// CHECK: This is the only method that is actually relevant for logging stuff, because the logger derives everything down to real attributes.
/// Since some of the asserts in the python notebooks are tied to the log output, the settings here do affect the notebook outcomes!!!
/// Since debug logging and the DataLogger output are different use-cases, the Logger should probably use a different method.
template <> String Attribute<Real>::toString() {
  return std::to_string(this->get());
}

template <> String Attribute<Complex>::toString() {
  std::stringstream ss;
  /// CHECK: Why do complex values only have precision 2, but reals have infinite precision?
  ss.precision(2);
  ss << this->get().real() << "+" << this->get().imag() << "i";
  return ss.str();
}

template <> String Attribute<String>::toString() { return this->get(); }

template class CPS::Attribute<Real>;
template class CPS::Attribute<Complex>;
template class CPS::Attribute<String>;
