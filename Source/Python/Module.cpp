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
#include "Python/Components.h"
#include "Python/LoadCim.h"
#include "Python/Interface.h"
#include "Python/Node.h"
#include "Python/SystemTopology.h"

using namespace DPsim;

static PyMethodDef dpsimModuleMethods[] = {
	{ "load_cim",               DPsim::Python::LoadCim,                                                                    METH_VARARGS, DPsim::Python::DocLoadCim },
	{ "open_interface",         (PyCFunction) DPsim::Python::OpenInterface,                                                METH_VARARGS|METH_KEYWORDS, DPsim::Python::DocOpenInterface },

	// Component constructors
	{ "CurrentSourceDP",        DPsim::Python::Components::CurrentSource<CPS::DP::Ph1::CurrentSource>,              METH_VARARGS, DPsim::Python::Components::DocCurrentSource },
	{ "CurrentSourceEMT",       DPsim::Python::Components::CurrentSource<CPS::EMT::Ph1::CurrentSource>,             METH_VARARGS, DPsim::Python::Components::DocCurrentSource },
	{ "VoltageSourceDP",        DPsim::Python::Components::VoltageSource<CPS::DP::Ph1::VoltageSource>,              METH_VARARGS, DPsim::Python::Components::DocVoltageSource },
	{ "VoltageSourceEMT",       DPsim::Python::Components::VoltageSource<CPS::EMT::Ph1::VoltageSource>,             METH_VARARGS, DPsim::Python::Components::DocVoltageSource },
	{ "VoltageSourceNortonDP",  DPsim::Python::Components::VoltageSourceNorton<CPS::DP::Ph1::VoltageSourceNorton>,  METH_VARARGS, DPsim::Python::Components::DocVoltageSourceNorton },
	{ "VoltageSourceNortonEMT", DPsim::Python::Components::VoltageSourceNorton<CPS::EMT::Ph1::VoltageSourceNorton>, METH_VARARGS, DPsim::Python::Components::DocVoltageSourceNorton },
	{ "InductorDP",             DPsim::Python::Components::Inductor<CPS::DP::Ph1::Inductor>,                        METH_VARARGS, DPsim::Python::Components::DocInductor },
	{ "InductorEMT",            DPsim::Python::Components::Inductor<CPS::EMT::Ph1::Inductor>,                       METH_VARARGS, DPsim::Python::Components::DocInductor },
	{ "ResistorDP",	            DPsim::Python::Components::Resistor<CPS::DP::Ph1::Resistor>,                        METH_VARARGS, DPsim::Python::Components::DocResistor },
	{ "ResistorEMT",            DPsim::Python::Components::Resistor<CPS::EMT::Ph1::Resistor>,                       METH_VARARGS, DPsim::Python::Components::DocResistor },
	{ "CapacitorDP",            DPsim::Python::Components::Capacitor<CPS::DP::Ph1::Capacitor>,                      METH_VARARGS, DPsim::Python::Components::DocCapacitor },
	{ "CapacitorEMT",           DPsim::Python::Components::Capacitor<CPS::EMT::Ph1::Capacitor>,                     METH_VARARGS, DPsim::Python::Components::DocCapacitor },
	{ "LinePiDP",               DPsim::Python::Components::LinePi<CPS::DP::Ph1::PiLine>,                            METH_VARARGS, DPsim::Python::Components::DocLinePi },
	{ "LoadPQDP",               DPsim::Python::Components::LoadPQ<CPS::DP::Ph1::PQLoad>,                            METH_VARARGS, DPsim::Python::Components::DocLoadPQ },
	{ 0 }
};

static PyModuleDef dpsimModule = {
	PyModuleDef_HEAD_INIT, "_dpsim", NULL, -1, dpsimModuleMethods, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit__dpsim(void) {
	PyObject* m;

	if (PyType_Ready(&DPsim::Python::ComponentType) < 0)
		return nullptr;
	if (PyType_Ready(&DPsim::Python::Node<CPS::Real>::type) < 0)
		return nullptr;
	if (PyType_Ready(&DPsim::Python::Node<CPS::Complex>::type) < 0)
		return nullptr;
	if (PyType_Ready(&DPsim::Python::SimulationType) < 0)
		return nullptr;
	if (PyType_Ready(&DPsim::Python::SystemTopologyType) < 0)
		return nullptr;
	if (PyType_Ready(&DPsim::Python::InterfaceType) < 0)
		return nullptr;

	m = PyModule_Create(&dpsimModule);
	if (!m)
		return nullptr;

	Py_INCREF(&DPsim::Python::SimulationType);
	PyModule_AddObject(m, "Simulation", (PyObject*) &DPsim::Python::SimulationType);
	Py_INCREF(&DPsim::Python::SystemTopologyType);
	PyModule_AddObject(m, "SystemTopology", (PyObject*) &DPsim::Python::SystemTopologyType);
	Py_INCREF(&DPsim::Python::ComponentType);
	PyModule_AddObject(m, "Component", (PyObject*) &DPsim::Python::ComponentType);
	
	Py_INCREF(&DPsim::Python::Node<CPS::Complex>::type);
	PyModule_AddObject(m, "DPNode", (PyObject*) &DPsim::Python::Node<CPS::Complex>::type);

	Py_INCREF(&DPsim::Python::Node<CPS::Real>::type);
	PyModule_AddObject(m, "EMTNode", (PyObject*) &DPsim::Python::Node<CPS::Real>::type);
	
	Py_INCREF(&DPsim::Python::InterfaceType);
	PyModule_AddObject(m, "Interface", (PyObject*) &DPsim::Python::InterfaceType);

	return m;
}
