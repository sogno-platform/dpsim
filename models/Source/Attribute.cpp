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
	ss << *mValue;
	return ss.str();
}

template<>
String Attribute<Matrix>::toString() const {
	std::stringstream ss;
	ss.precision(2);
	ss << *mValue;
	return ss.str();
}

template<>
String Attribute<Complex>::toString() const {
	std::stringstream ss;
	ss.precision(2);
	ss << mValue->real() << "+" << mValue->imag() << "i";
	return ss.str();
}

template<>
String Attribute<String>::toString() const {
	return String(*mValue);
}

#ifdef WITH_PYTHON

// Matrix
template<>
void Attribute<Matrix>::fromPyObject(PyObject *po) {
	throw std::runtime_error("not implemented"); // TODO
}

template<>
PyObject * Attribute<Matrix>::toPyObject() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// MatrixComp
template<>
void Attribute<MatrixComp>::fromPyObject(PyObject *po) {
	throw std::runtime_error("not implemented"); // TODO
}

template<>
PyObject * Attribute<MatrixComp>::toPyObject() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// Int
template<>
void Attribute<Int>::fromPyObject(PyObject *po) {
	Int i = PyLong_AsLong(po);
	if (PyErr_Occurred())
		throw TypeException();

	set(i);
}

template<>
PyObject * Attribute<Int>::toPyObject() {
	Int i = get();
	return PyLong_FromLong(i);
}

// UInt
template<>
void Attribute<UInt>::fromPyObject(PyObject *po) {
	UInt i = PyLong_AsUnsignedLong(po);
	if (PyErr_Occurred())
		throw TypeException();

	set(i);
}

template<>
PyObject * Attribute<UInt>::toPyObject() {
	UInt i = get();
	return PyLong_FromUnsignedLong(i);
}

// Real
template<>
void Attribute<Real>::fromPyObject(PyObject *po) {
	Real r = PyFloat_AsDouble(po);
	if (PyErr_Occurred())
		throw TypeException();

	set(r);
}

template<>
PyObject * Attribute<Real>::toPyObject() {
	Real r = get();
	return PyFloat_FromDouble(r);
}

// Complex
template<>
void Attribute<Complex>::fromPyObject(PyObject *po) {
	if (PyComplex_Check(po)) {
		Complex c = Complex(PyComplex_RealAsDouble(po), PyComplex_ImagAsDouble(po));
		set(c);
	} else {
		Real r = PyFloat_AsDouble(po);
		if (PyErr_Occurred())
			throw TypeException();
		set(Complex(r, 0));
	}
}

template<>
PyObject * Attribute<Complex>::toPyObject() {
	Complex c = get();
	return PyComplex_FromDoubles(c.real(), c.imag());
}

// String
template<>
void Attribute<String>::fromPyObject(PyObject *po) {
	if (!PyUnicode_Check(po))
		throw TypeException();

	String s = PyUnicode_AsUTF8(po);
	set(s);
}

template<>
PyObject * Attribute<String>::toPyObject() {
	String s = get();
	return PyUnicode_FromString(s.c_str());
}

// Bool
template<>
void Attribute<Bool>::fromPyObject(PyObject *po) {
	Int r = PyObject_IsTrue(po);
	if (PyErr_Occurred())
		throw TypeException();

	set(r);
}

template<>
PyObject * Attribute<Bool>::toPyObject() {
	PyObject *b = get() ? Py_True : Py_False;

	Py_INCREF(b);
	return b;
}

#endif // WITH_PYTHON

template class CPS::Attribute<MatrixComp>;
template class CPS::Attribute<Matrix>;
template class CPS::Attribute<Complex>;
template class CPS::Attribute<String>;
template class CPS::Attribute<Int>;
template class CPS::Attribute<Real>;
