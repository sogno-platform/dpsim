/** DPsim.
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include "Config.h"
#include "Utils.h"
#include "Simulation.h"

#ifdef WITH_RT
  #include "RealTimeSimulation.h"
#endif

#include "cps/Components.h"
#include "cps/Logger.h"

#ifdef WITH_SHMEM
  #include "cps/Interface.h"
#endif

#ifdef WITH_CIM
  #include "cps/CIM/Reader.h"
#endif
