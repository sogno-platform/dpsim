/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
