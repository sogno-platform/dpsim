/** Python Logger
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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
