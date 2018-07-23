/** Python components
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Config.h"
#include "Python/LoadCim.h"
#include "Python/Component.h"

#include "cps/Components.h"
#include "cps/SystemTopology.h"
#include "Python/SystemTopology.h"

#ifdef WITH_CIM
  #include "cps/CIM/Reader.h"
#endif

using namespace CPS;

const char* Python::DocLoadCim =
"load_cim(filenames, frequency=50.0)\n"
"Load a network from CIM file(s).\n"
"\n"
":param filenames: Either a filename or a list of filenames of CIM files to be loaded.\n"
":param frequency: Nominal system frequency in Hz.\n"
":returns: A list of `dpsim.Component`.\n"
"\n"
"Note that in order for the CIM parser to function properly, the CSV "
"files containing the alias configuration have to be in the working directory "
"of the program.\n";
PyObject* Python::LoadCim(PyObject* self, PyObject* args) {
#ifdef WITH_CIM
	Real frequency = 50;
	PyObject *filenames;
	PyBytesObject *filename;
	std::list<String> cimFiles;

	if (PyArg_ParseTuple(args, "O&|d", PyUnicode_FSConverter, &filename, &frequency)) {
		cimFiles.push_back(PyBytes_AsString((PyObject*) filename));
		Py_DECREF(filename);
	}
	else if (PyArg_ParseTuple(args, "O|d", &filenames, &frequency)) {
		PyErr_Clear();

		if (PyList_Check(filenames)) {
			for (int i = 0; i < PyList_Size(filenames); i++) {
				if (!PyUnicode_FSConverter(PyList_GetItem(filenames, i), &filename)) {
					PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
					return nullptr;
				}
				cimFiles.push_back(PyBytes_AsString((PyObject*) filename));
				Py_DECREF(filename);
			}
		}
		Py_DECREF(filenames);
	}

	if (cimFiles.size() == 0) {
		PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
		return nullptr;
	}

	CIM::Reader reader("Python", Logger::Level::INFO);

	CPS::Python::SystemTopology *pySys = PyObject_New(CPS::Python::SystemTopology, &CPS::Python::SystemTopologyType);

	using SharedSysPtr = std::shared_ptr<CPS::SystemTopology>;
	using PyObjectsList = std::vector<PyObject *>;

	new (&pySys->sys) SharedSysPtr();
	new (&pySys->refs) PyObjectsList();

	try {
		pySys->sys = std::make_shared<CPS::SystemTopology>(reader.loadCIM(frequency, cimFiles));
	}
	catch (CIM::InvalidTopology) {
		PyErr_SetString(PyExc_TypeError, "The topology of the CIM model is invalid");
		return nullptr;
	}

	Py_INCREF(pySys);

	return (PyObject *) pySys;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}
