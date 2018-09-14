/** Python module
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <dpsim/Config.h>
#include <dpsim/Python/Module.h>
#include <dpsim/Python/Component.h>
#include <dpsim/Python/Node.h>
#include <dpsim/Python/SystemTopology.h>
#include <dpsim/Python/Simulation.h>
#include <dpsim/Python/Interface.h>
#include <dpsim/Python/LoadCim.h>

#include <cps/Components.h>

using namespace DPsim::Python;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim",               (PyCFunction) LoadCim,       METH_VARARGS | METH_KEYWORDS, DPsim::Python::DocLoadCim },

	// Component constructors
	#include <dpsim/Python/ComponentConstructors.h>

	{ nullptr }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "_dpsim", nullptr, -1, dpsimModuleMethods, nullptr, nullptr, nullptr, nullptr
};

PyMODINIT_FUNC PyInit__dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&Component::type) < 0)
		return nullptr;
	if (PyType_Ready(&Node<CPS::Real>::type) < 0)
		return nullptr;
	if (PyType_Ready(&Node<CPS::Complex>::type) < 0)
		return nullptr;
	if (PyType_Ready(&Simulation::type) < 0)
		return nullptr;
	if (PyType_Ready(&SystemTopology::type) < 0)
		return nullptr;
	if (PyType_Ready(&Interface::type) < 0)
		return nullptr;

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	Py_INCREF(&Simulation::type);
	PyModule_AddObject(m, "Simulation", (PyObject*) &Simulation::type);
	Py_INCREF(&SystemTopology::type);
	PyModule_AddObject(m, "SystemTopology", (PyObject*) &SystemTopology::type);
	Py_INCREF(&Component::type);
	PyModule_AddObject(m, "Component", (PyObject*) &Component::type);
	Py_INCREF(&Interface::type);
	PyModule_AddObject(m, "Interface", (PyObject*) &Interface::type);
	Py_INCREF(&Node<CPS::Complex>::type);
	PyModule_AddObject(m, "_dp_Node", (PyObject*) &Node<CPS::Complex>::type);
	Py_INCREF(&Node<CPS::Real>::type);
	PyModule_AddObject(m, "_emt_Node", (PyObject*) &Node<CPS::Real>::type);

	return m;
}
