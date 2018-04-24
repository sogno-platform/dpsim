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
#include "Python/Simulation.h"

#include "cps/Components.h"
#include "cps/Python/Components.h"
#include "cps/Python/LoadCim.h"
#include "cps/Python/Interface.h"
#include "cps/Python/Node.h"
#include "cps/Python/SystemTopology.h"

using namespace DPsim;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim",               CPS::Python::LoadCim, METH_VARARGS, CPS::Python::DocLoadCim },
	{ "open_shmem_interface",   (PyCFunction) CPS::Python::OpenShmemInterface, METH_VARARGS|METH_KEYWORDS, CPS::Python::DocOpenShmemInterface },

	// Component constructors
	{ "CurrentSourceDP",        CPS::Python::Components::CurrentSource<CPS::Components::DP::CurrentSource>,              METH_VARARGS, CPS::Python::Components::DocCurrentSource },
	{ "CurrentSourceEMT",       CPS::Python::Components::CurrentSource<CPS::Components::EMT::CurrentSource>,             METH_VARARGS, CPS::Python::Components::DocCurrentSource },
	{ "VoltageSourceDP",        CPS::Python::Components::VoltageSource<CPS::Components::DP::VoltageSource>,              METH_VARARGS, CPS::Python::Components::DocVoltageSource },
	{ "VoltageSourceEMT",       CPS::Python::Components::VoltageSource<CPS::Components::EMT::VoltageSource>,             METH_VARARGS, CPS::Python::Components::DocVoltageSource },
	{ "VoltageSourceNortonDP",  CPS::Python::Components::VoltageSourceNorton<CPS::Components::DP::VoltageSourceNorton>,  METH_VARARGS, CPS::Python::Components::DocVoltageSourceNorton },
	{ "VoltageSourceNortonEMT", CPS::Python::Components::VoltageSourceNorton<CPS::Components::EMT::VoltageSourceNorton>, METH_VARARGS, CPS::Python::Components::DocVoltageSourceNorton },
	{ "InductorDP",             CPS::Python::Components::Inductor<CPS::Components::DP::Inductor>,                        METH_VARARGS, CPS::Python::Components::DocInductor },
	{ "InductorEMT",            CPS::Python::Components::Inductor<CPS::Components::EMT::Inductor>,                       METH_VARARGS, CPS::Python::Components::DocInductor },
	{ "ResistorDP",	            CPS::Python::Components::Resistor<CPS::Components::DP::Resistor>,                        METH_VARARGS, CPS::Python::Components::DocResistor },
	{ "ResistorEMT",            CPS::Python::Components::Resistor<CPS::Components::EMT::Resistor>,                       METH_VARARGS, CPS::Python::Components::DocResistor },
	{ "CapacitorDP",            CPS::Python::Components::Capacitor<CPS::Components::DP::Capacitor>,                      METH_VARARGS, CPS::Python::Components::DocCapacitor },
	{ "CapacitorEMT",           CPS::Python::Components::Capacitor<CPS::Components::EMT::Capacitor>,                     METH_VARARGS, CPS::Python::Components::DocCapacitor },
	{ "LinePiDP",               CPS::Python::Components::LinePi<CPS::Components::DP::PiLine>,                            METH_VARARGS, CPS::Python::Components::DocLinePi },
	{ "LineRxDP",               CPS::Python::Components::LineRx<CPS::Components::DP::RxLine>,                            METH_VARARGS, CPS::Python::Components::DocLineRx },
	{ "LoadPQDP",               CPS::Python::Components::LoadPQ<CPS::Components::DP::PQLoad>,                            METH_VARARGS, CPS::Python::Components::DocLoadPQ },
	{ 0 }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "_dpsim", NULL, -1, dpsimModuleMethods, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit__dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&CPS::Python::ComponentType) < 0)
		return nullptr;
	if (PyType_Ready(&CPS::Python::NodeType) < 0)
		return nullptr;
	if (PyType_Ready(&DPsim::Python::SimulationType) < 0)
		return nullptr;
	if (PyType_Ready(&CPS::Python::SystemTopologyType) < 0)
		return nullptr;
	CPS::Python::InterfaceType.tp_new = PyType_GenericNew;
	if (PyType_Ready(&CPS::Python::InterfaceType) < 0)
		return nullptr;

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	Py_INCREF(&DPsim::Python::SimulationType);
	PyModule_AddObject(m, "Simulation", (PyObject*) &DPsim::Python::SimulationType);
	Py_INCREF(&CPS::Python::SystemTopologyType);
	PyModule_AddObject(m, "SystemTopology", (PyObject*) &CPS::Python::SystemTopologyType);
	Py_INCREF(&CPS::Python::ComponentType);
	PyModule_AddObject(m, "Component", (PyObject*) &CPS::Python::ComponentType);
	Py_INCREF(&CPS::Python::NodeType);
	PyModule_AddObject(m, "Node", (PyObject*) &CPS::Python::NodeType);
	Py_INCREF(&CPS::Python::InterfaceType);
	PyModule_AddObject(m, "Interface", (PyObject*) &CPS::Python::InterfaceType);

	return m;
}
