/** Python Compatability for Attributes
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

#include <cps/Config.h>
#include <cps/Attribute.h>

#include <numpy/arrayobject.h>

namespace CPS {

// Matrix
template<>
PyArray_Descr * Attribute<Matrix>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_DOUBLE);
}

template<>
PyObject * Attribute<Matrix>::toPyArray() {
	auto &v = get();

	npy_intp dims[] = { v.rows(), v.cols() };

	return PyArray_NewFromDescr(&PyArray_Type, toPyArrayDescr(), 2, dims, nullptr, nullptr, 0, nullptr);
}

// MatrixComp
template<>
PyArray_Descr * Attribute<MatrixComp>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_CDOUBLE);
}

template<>
PyObject * Attribute<MatrixComp>::toPyArray() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// Int
template<>
PyArray_Descr * Attribute<Int>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_INT);
}

template<>
PyObject * Attribute<Int>::toPyArray() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// UInt
template<>
PyArray_Descr * Attribute<UInt>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_UINT);
}

template<>
PyObject * Attribute<UInt>::toPyArray() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// Real
template<>
PyArray_Descr * Attribute<Real>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_DOUBLE);
}

template<>
PyObject * Attribute<Real>::toPyArray() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// Complex
template<>
PyArray_Descr * Attribute<Complex>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_CDOUBLE);
}

template<>
PyObject * Attribute<Complex>::toPyArray() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

// Bool
template<>
PyArray_Descr * Attribute<Bool>::toPyArrayDescr() {
	return PyArray_DescrFromType(NPY_BOOL);
}

template<>
PyObject * Attribute<Bool>::toPyArray() {
	throw std::runtime_error("not implemented"); // TODO

	return nullptr;
}

}
