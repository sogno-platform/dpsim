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

#include <dpsim/Config.h>
#include <dpsim/Python/LoadCim.h>
#include <dpsim/Python/Component.h>

#include <cps/SystemTopology.h>
#include <cps/Logger.h>
#include <dpsim/Python/SystemTopology.h>

#ifdef WITH_CIM
  #include <cps/CIM/Reader.h>
#endif

using namespace DPsim;

const char* Python::DocLoadCim =
"load_cim(name, filenames, frequency=50.0, log_level=0)\n"
"Load a network from CIM file(s).\n"
"\n"
":param filenames: Either a filename or a list of filenames of CIM files to be loaded.\n"
":param frequency: Nominal system frequency in Hz.\n"
":returns: A list of `dpsim.Component`.\n"
"\n"
"Note that in order for the CIM parser to function properly, the CSV "
"files containing the alias configuration have to be in the working directory "
"of the program.\n";
PyObject* Python::LoadCim(PyObject* self, PyObject* args, PyObject *kwargs) {
#ifdef WITH_CIM
	CPS::Real frequency = 50;
	PyObject *filenames;
	PyBytesObject *filename;
	std::list<String> cimFiles;
	int logLevel = (int) CPS::Logger::Level::INFO;
	const char *name;

	const char *kwlist[] = {"name", "files", "frequency", "log_level", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwargs, "sO|di", (char **) kwlist, &name, &filenames, &frequency, &logLevel))
		return nullptr;

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
	else if (PyBytes_Check(filenames)) {
		if (!PyUnicode_FSConverter(filenames, &filename)) {
			PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
			return nullptr;
		}
		cimFiles.push_back(PyBytes_AsString((PyObject*) filename));
	}
	else {
		PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
		return nullptr;
	}

	Py_DECREF(filenames);

	if (cimFiles.size() == 0) {
		PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
		return nullptr;
	}

	CPS::CIM::Reader reader(name, (CPS::Logger::Level) logLevel, (CPS::Logger::Level) logLevel);

	DPsim::Python::SystemTopology *pySys = PyObject_New(DPsim::Python::SystemTopology, &DPsim::Python::SystemTopology::type);

	using SharedSysPtr = std::shared_ptr<CPS::SystemTopology>;

	new (&pySys->sys) SharedSysPtr();
	pySys->pyComponentDict = PyDict_New();
	pySys->pyNodeDict = PyDict_New();

	try {
		pySys->sys = std::make_shared<CPS::SystemTopology>(reader.loadCIM(frequency, cimFiles));
	}
	catch (const CPS::CIM::InvalidTopology &) {
		PyErr_SetString(PyExc_TypeError, "The topology of the CIM model is invalid");
		return nullptr;
	}

	// Fill pyComponentDict, pyNodeDict
	for (auto comp : pySys->sys->mComponents) {
		PyObject *pyName = PyUnicode_FromString(comp->name().c_str());

		if (PyDict_Contains(pySys->pyComponentDict, pyName)) {
			PyErr_Format(PyExc_AttributeError, "Duplicated Component name");
			return nullptr;
		}

		Component *pyComp = PyObject_New(Component, &Component::type);
		Component::init(pyComp);

		pyComp->comp = comp;

		PyDict_SetItem(pySys->pyComponentDict, pyName, (PyObject *) pyComp);
	}

	for (auto node : pySys->sys->mNodes) {
		PyObject *pyName = PyUnicode_FromString(node->name().c_str());

		if (PyDict_Contains(pySys->pyNodeDict, pyName)) {
			PyErr_Format(PyExc_AttributeError, "Duplicated Node name");
			return nullptr;
		}

		auto dpNode = std::dynamic_pointer_cast<CPS::Node<CPS::Complex>>(node);
		if (dpNode) {
			Python::Node<CPS::Complex> *pyNode = PyObject_New(Python::Node<CPS::Complex>, &Python::Node<CPS::Complex>::type);

			using SharedNodePtr = std::shared_ptr<CPS::Node<CPS::Complex>>;
			new (&pyNode->node) SharedNodePtr();

			pyNode->node = dpNode;

			PyDict_SetItem(pySys->pyNodeDict, pyName, (PyObject*) pyNode);
			continue;
		}

		auto emtNode = std::dynamic_pointer_cast<CPS::Node<CPS::Real>>(node);
		if (emtNode) {
			Python::Node<CPS::Real> *pyNode = PyObject_New(Python::Node<CPS::Real>, &Python::Node<CPS::Real>::type);

			using SharedNodePtr = std::shared_ptr<CPS::Node<CPS::Real>>;
			new (&pyNode->node) SharedNodePtr();

			pyNode->node = emtNode;

			PyDict_SetItem(pySys->pyNodeDict, pyName, (PyObject*) pyNode);
			continue;
		}

		return nullptr;
	}

	Py_INCREF(pySys);
	return (PyObject *) pySys;
#else
	PyErr_SetString(PyExc_NotImplementedError, "not implemented on this platform");
	return nullptr;
#endif
}
