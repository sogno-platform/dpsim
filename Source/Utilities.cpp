/** Utilities
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

#include "Utilities.h"

#include <iostream>

void updateProgressBar(double time, double finalTime) {

	if (time / finalTime <= 0.1) {
		std::cout << "                      (0%)\r";
	}
	else if (time / finalTime > 0.1 && time / finalTime <= 0.2) {
		std::cout << "##                    (10%)\r";
	}
	else if (time / finalTime > 0.2 && time / finalTime <= 0.3) {
		std::cout << "####                  (20%)\r";
	}
	else if (time / finalTime > 0.3 && time / finalTime <= 0.4) {
		std::cout << "######                (30%)\r";
	}
	else if (time / finalTime > 0.4 && time / finalTime <= 0.5) {
		std::cout << "########              (40%)\r";
	}
	else if (time / finalTime > 0.5 && time / finalTime <= 0.6) {
		std::cout << "##########            (50%)\r";
	}
	else if (time / finalTime > 0.6 && time / finalTime <= 0.7) {
		std::cout << "############          (60%)\r";
	}
	else if (time / finalTime > 0.7 && time / finalTime <= 0.8) {
		std::cout << "##############        (70%)\r";
	}
	else if (time / finalTime > 0.8 && time / finalTime <= 0.9) {
		std::cout << "################      (80%)\r";
	}
	else if (time / finalTime > 0.9 && time / finalTime < 1) {
		std::cout << "##################    (90%)\r";
	}
	else {
		std::cout << "####################  (100%)";
		std::cout << std::endl;
	}
}
