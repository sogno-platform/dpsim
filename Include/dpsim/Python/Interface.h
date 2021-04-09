/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <dpsim/Interface.h>

namespace DPsim {

namespace Python {

	// Thin Python wrapper around Interface
	struct Interface {
		PyObject_HEAD

		static void addExportDesc(Interface *self, int idx, const CPS::String &type, const CPS::String &name, const CPS::String &suffix = "");

		static void dealloc(Interface *self);

		static int init(Interface *self, PyObject *args, PyObject *kwds);
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static PyObject* addImport(Interface *self, PyObject *args, PyObject *kwargs);
		static PyObject* addExport(Interface *self, PyObject *args, PyObject *kwargs);
		static PyObject* exports(Interface *self, void *ctx);

		static PyMethodDef methods[];
		static PyMemberDef members[];
		static PyTypeObject type;
		static const char* doc;
		static const char* docOpen;
		static const char* docAddImport;
		static const char* docAddExport;
	};
}
}
