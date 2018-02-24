/** Attributes
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

#include "Attribute.h"

using namespace DPsim;

#ifdef WITH_PYTHON

/* Specialization for Python Attributes */

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

template<>
void Attribute<Complex>::fromPyObject(PyObject *po) {
	if (PyComplex_Check(po))
		throw TypeException();

	Complex c = Complex(PyComplex_RealAsDouble(po), PyComplex_ImagAsDouble(po));

	set(c);
}

template<>
PyObject * Attribute<Complex>::toPyObject() {
	Complex c = get();

	return PyComplex_FromDoubles(c.real(), c.imag());
}

template<>
void Attribute<String>::fromPyObject(PyObject *po) {
	if (!PyUnicode_Check(po))
		throw TypeException();

	String s = PyUnicode_AsUTF8(po);

	set(s);
}

#include <iostream>

template<>
PyObject * Attribute<String>::toPyObject() {
	String s = get();

	return PyUnicode_FromString(s.c_str());
}

#endif
