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

#include <dpsim/Config.h>
#include <cps/SystemTopology.h>

namespace DPsim {
namespace Python {

	struct SystemTopology {
		PyObject_HEAD

		CPS::SystemTopology::Ptr sys;

		PyObject *pyNodeDict;
		PyObject *pyComponentDict;

		// Helper functions to update the python dicts after a C++ method
		// adds components or nodes.
		void addCppComponent(CPS::IdentifiedObject::Ptr comp);
		void addCppNode(CPS::TopologicalNode::Ptr node);
		void updateDicts();

		static PyObject* addComponent(SystemTopology *self, PyObject *args);
		static PyObject* addNode(SystemTopology *self, PyObject *args);
		static PyObject* addDecouplingLine(SystemTopology *self, PyObject *args);
		static PyObject* autoDecouple(SystemTopology *self, PyObject *args);
		static PyObject* multiply(SystemTopology *self, PyObject *args);
		static PyObject* removeComponent(SystemTopology *self, PyObject *args);
#ifdef WITH_GRAPHVIZ
		static PyObject* reprSVG(SystemTopology *self, PyObject *args);
		static const char *docReprSVG;
#endif

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static int init(SystemTopology *self, PyObject *args, PyObject *kwds);
		static void dealloc(SystemTopology *self);

		static const char *doc;
		static const char *docNodes;
		static const char *docComponents;
		static const char *docAddComponent;
		static const char *docAddNode;
		static const char *docAddDecouplingLine;
		static const char *docAutoDecouple;
		static const char *docRemoveComponent;
		static const char *docMultiply;
		static PyTypeObject type;
		static PyMethodDef methods[];
		static PyMemberDef members[];
	};
}
}
