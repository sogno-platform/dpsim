/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>

#include <cps/Config.h>
#include <cps/Attribute.h>

using namespace CPS;

std::ostream &operator<<(std::ostream &output, const AttributeBase &attr) {
	output << attr.toString();
	return output;
}

template<>
String Attribute<MatrixComp>::toString() const {
	std::stringstream ss;
	ss.precision(2);
	ss << *mData;
	return ss.str();
}

template<>
String Attribute<Matrix>::toString() const {
	std::stringstream ss;
	ss.precision(2);
	ss << *this->get();
	return ss.str();
}

template<>
String Attribute<Complex>::toString() const {
	std::stringstream ss;
	ss.precision(2);
	ss << this->get()->real() << "+" << this->get()->imag() << "i";
	return ss.str();
}

template<>
String Attribute<String>::toString() const {
	return String(*this->get());
}

template class CPS::Attribute<MatrixComp>;
template class CPS::Attribute<Matrix>;
template class CPS::Attribute<Complex>;
template class CPS::Attribute<String>;
template class CPS::Attribute<Int>;
template class CPS::Attribute<Real>;
