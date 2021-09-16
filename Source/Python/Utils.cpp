/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <ctime>

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <dpsim/Python/Utils.h>

using namespace DPsim::Python;

void DPsim::Python::setAttributes(CPS::AttributeList::Ptr al, PyObject *kwargs)
{
	PyObject *key, *value;
	Py_ssize_t pos = 0;

	while (PyDict_Next(kwargs, &pos, &key, &value)) {
		CPS::String name = PyUnicode_AsUTF8(key);

		auto attr = al->attribute(name);
		attr->fromPyObject(value);
	}
}
