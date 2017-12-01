/** The main() routine of the DPsim Examples
*
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include <iostream>

#include "ShmemTest.h"

#include "CIMTest.h"

using namespace DPsim;

int main(int argc, char* argv[]) {

	// #### CIM Parser test ################
	//readFixedCIMFiles_LineLoad();
	readFixedCIMFiles_IEEE9bus();

	// #### Shared memory interface tests ################
	//shmemRTExample();
	//shmemDistributed(argc, argv);
	//shmemDistributedRef();

	//std::cin.get();
	return 0;
}

