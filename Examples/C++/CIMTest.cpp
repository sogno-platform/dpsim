/** CIM Test
 *
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
#include <string>

#include "CIMTest.h"
#include "CIMReader.h"
#include "Simulation.h"

void readFixedCIMFiles_LineLoad() {
	std::list<std::string> filenames;
	filenames.push_back("..\\..\\Examples\\CIM\\Line_Load\\Line_Load.xml");
	testCIMReader(filenames);
}

void readFixedCIMFiles_IEEE9bus() {
	std::list<std::string> filenames;
	filenames.push_back("..\\..\\Examples\\CIM\\IEEE-09_Neplan_RX\\IEEE-09_Neplan_RX_DI.xml");
	filenames.push_back("..\\..\\Examples\\CIM\\IEEE-09_Neplan_RX\\IEEE-09_Neplan_RX_EQ.xml");
	filenames.push_back("..\\..\\Examples\\CIM\\IEEE-09_Neplan_RX\\IEEE-09_Neplan_RX_SV.xml");
	filenames.push_back("..\\..\\Examples\\CIM\\IEEE-09_Neplan_RX\\IEEE-09_Neplan_RX_TP.xml");
	testCIMReader(filenames);
}

int testCIMReader(std::list<std::string> filenames) {
	
	// Read CIM data
	CIMReader reader(50);
	Logger log("cim.log"), llog("lvector-cim.csv"), rlog("rvector-cim.csv");

	for (std::string & filename : filenames) {
		if (!reader.addFile(filename))
			std::cerr << "Failed to read file " << filename << std::endl;
	}

	reader.parseFiles();

	std::vector<BaseComponent*> components = reader.getComponents();
	
	// Run simulation as usually
	Simulation sim(components, 2 * PI * 50, 0.001, 0.3, log);
	std::cout << "Start simulation." << std::endl;
	while (sim.step(log, llog, rlog))
		sim.increaseByTimeStep();
	std::cout << "Simulation finished." << std::endl;
	
	return 0;
}
