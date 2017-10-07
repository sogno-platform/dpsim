/** Python module
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

#include <Python.h>

namespace DPsim {
	extern PyMethodDef pyModuleMethods[];

	extern PyModuleDef dpsimModule;

};

// Has to be declared as extern C and without a namespace, because  the linker
// otherwise mangles the name so the Python interpreter can't find this function.
extern "C" {

#if(_WIN32 || _WIN64)
	extern __declspec(dllexport) PyObject* PyInit_dpsim(void);
#else
	extern PyObject* PyInit_dpsim(void);
#endif
};
