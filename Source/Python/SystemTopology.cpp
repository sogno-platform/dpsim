/** Python system tolopogy
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

#include "SystemTopology.h"

using namespace DPsim;

static PyMethodDef SystemTopology_methods[] = {
	{NULL, NULL, 0, NULL}
};

static const char* DocSystemTopology =
"A system topology.\n"
"\n"
"Proper ``__init__`` signature:\n"
"\n"
"``__init__(self, frequency=50.0, components)``.\n\n"
"``frequency`` is the nominal system frequency in Hz.\n\n"
"``components`` must be a list of `Component` that are to be simulated.\n\n";
PyTypeObject DPsim::Python::SystemTopologyType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.System",                          /* tp_name */
	sizeof(Python::SystemTopology),          /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)Python::SystemTopology::dealloc, /* tp_dealloc */
	0,                                       /* tp_print */
	0,                                       /* tp_getattr */
	0,                                       /* tp_setattr */
	0,                                       /* tp_reserved */
	0,                                       /* tp_repr */
	0,                                       /* tp_as_number */
	0,                                       /* tp_as_sequence */
	0,                                       /* tp_as_mapping */
	0,                                       /* tp_hash  */
	0,                                       /* tp_call */
	0,                                       /* tp_str */
	0,                                       /* tp_getattro */
	0,                                       /* tp_setattro */
	0,                                       /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,/* tp_flags */
	DocSystemTopology,                       /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	SystemTopology_methods,                  /* tp_methods */
	0,                                       /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc)Python::System::init,          /* tp_init */
	0,                                       /* tp_alloc */
	Python::System::newfunc,                 /* tp_new */
};
