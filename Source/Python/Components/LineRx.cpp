/** Python binding for Capacitors.
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
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

#include "Python/Components/LineRx.h"

const char *DPsim::Python::Components::DocLineRx =
"LineRx(name, node1, node2, resistance, inductance, capacitance)\n"
"Construct a new Rx line.\n"
"\n"
"Attributes: ``resistance``, ``inductance``, ``capacitance``.\n"
"\n"
":param resistance: Resistance in Ohm.\n"
":param inductance: Inductance in Henry.\n"
":param capacitance: Capacitance in Farad.\n"
":returns: A new `Component` representing this Pi line.\n";
