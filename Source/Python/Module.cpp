/** Python module
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include "Config.h"
#include "Python/Module.h"
#include "Python/Component.h"
#include "Python/Node.h"
#include "Python/SystemTopology.h"
#include "Python/Simulation.h"
#include "Python/Interface.h"
#include "Python/LoadCim.h"

using namespace DPsim::Python;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim",               LoadCim,                                                            METH_VARARGS, DPsim::Python::DocLoadCim },
	{ "open_interface",         (PyCFunction) OpenInterface,                                        METH_VARARGS | METH_KEYWORDS, DPsim::Python::DocOpenInterface },

	// Component constructors
	#include "Python/ComponentConstructors.h"

	{ 0 }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "_dpsim", NULL, -1, dpsimModuleMethods, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit__dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&ComponentType) < 0)
		return nullptr;
	if (PyType_Ready(&Node<CPS::Real>::type) < 0)
		return nullptr;
	if (PyType_Ready(&Node<CPS::Complex>::type) < 0)
		return nullptr;
	if (PyType_Ready(&SimulationType) < 0)
		return nullptr;
	if (PyType_Ready(&SystemTopologyType) < 0)
		return nullptr;
	if (PyType_Ready(&InterfaceType) < 0)
		return nullptr;

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	Py_INCREF(&SimulationType);
	PyModule_AddObject(m, "Simulation", (PyObject*) &SimulationType);
	Py_INCREF(&SystemTopologyType);
	PyModule_AddObject(m, "SystemTopology", (PyObject*) &SystemTopologyType);
	Py_INCREF(&ComponentType);
	PyModule_AddObject(m, "Component", (PyObject*) &ComponentType);
	Py_INCREF(&InterfaceType);
	PyModule_AddObject(m, "Interface", (PyObject*) &InterfaceType);
	Py_INCREF(&Node<CPS::Complex>::type);
	PyModule_AddObject(m, "_dp_Node", (PyObject*) &Node<CPS::Complex>::type);
	Py_INCREF(&Node<CPS::Real>::type);
	PyModule_AddObject(m, "_emt_Node", (PyObject*) &Node<CPS::Real>::type);

	return m;
}
