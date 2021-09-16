/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <cps/SimNode.h>

namespace DPsim {
namespace Python {

	template<typename VarType>
	struct Node {
		PyObject_HEAD

		typename CPS::SimNode<VarType>::Ptr node;

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static int init(Node<VarType> *self, PyObject *args, PyObject *kwds);
		static void dealloc(Node<VarType> *self);

		static PyObject* initialVoltage(PyObject *self, PyObject *args);
		static PyObject* voltage(PyObject *self, PyObject *args);
		static PyObject* gnd(PyObject *self, PyObject *args);

		static PyObject* setInitialVoltage(PyObject *self, PyObject *args, PyObject *kwds);

		static const char *name;
		static const char *doc;
		static const char *docInitialVoltage;
		static const char *docVoltage;
		static const char *docGND;
		static const char *docSetInitialVoltage;
		static PyMethodDef methods[];
		static PyTypeObject type;

		static PyObject *Py_GND;

		static typename CPS::SimNode<VarType>::List fromPython(PyObject* list);
	};

	template<typename VarType>
	PyObject *Node<VarType>::Py_GND = NULL;

#ifdef _WIN32
	//template PyTypeObject Node<CPS::Real>::type;
	//template PyTypeObject Node<CPS::Complex>::type;
#else
	extern template PyTypeObject Node<CPS::Real>::type;
	extern template PyTypeObject Node<CPS::Complex>::type;
#endif

	CPS::TopologicalNode::List nodesFromPython(PyObject* list);
}
}
