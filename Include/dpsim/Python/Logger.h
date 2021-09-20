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

#include <cps/Definitions.h>

namespace DPsim {
	class DataLogger;

namespace Python {

	// Thin Python wrapper around Logger
	struct Logger {
		PyObject_HEAD

		std::vector<PyObject *> refs;

		DPsim::DataLogger::Ptr logger;
		const char *filename;

		static void dealloc(Logger *self);

		static int init(Logger *self, PyObject *args, PyObject *kwds);
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static PyObject* logAttribute(Logger *self, PyObject *args, PyObject *kwargs);

		static PyMethodDef methods[];
		static PyMemberDef members[];
		static PyTypeObject type;
		static const char* doc;
		static const char* docLogAttribute;
	};
}
}
