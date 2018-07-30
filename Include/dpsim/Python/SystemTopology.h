/** Python system tolopogy
 *
 * @file
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

#pragma once

#include <Config.h>
#include <cps/SystemTopology.h>

namespace DPsim {
namespace Python {

	struct SystemTopology {
		PyObject_HEAD

		std::shared_ptr<CPS::SystemTopology> sys;

		// List of additional objects that aren't directly used from Simulation
		// methods, but that a reference has be kept to to avoid them from being
		// freed (e.g. Interfaces).
		std::vector<PyObject*> refs;

		static PyObject* addComponent(PyObject *self, PyObject *args);
#ifdef WITH_GRAPHVIZ
		static PyObject* reprSVG(PyObject *self, PyObject *args);
#endif

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static int init(SystemTopology *self, PyObject *args, PyObject *kwds);
		static void dealloc(SystemTopology *self);
	};

	extern PyTypeObject SystemTopologyType;
}
}
