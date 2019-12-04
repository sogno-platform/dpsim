/** Attributes
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
