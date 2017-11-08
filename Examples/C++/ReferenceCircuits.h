/** Reference Circuits
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

#pragma once

#include "Definitions.h"

namespace DPsim {
	// Simulink models can be found in the ReferenceExamples repository
	void simulationExample1();
	void simulationExample1L2();
	void simulationExample2();
	void simulationExample3();
	void simulationExampleIdealVS();
	void simulationExampleIdealVS2();
	void simulationExampleIdealVS3();
	void simulationExampleRXLine();
	void simulationExampleRXLine2();
	void simulationExampleRXLine3();
	void simulationExamplePiLine();
	void simulationExamplePiLine2();
	void simulationExampleTestIdealVoltageSourceEMT();
	
	// Circuits used for ICCEP2017 paper
	void RXLineResLoad();
	void VarFreqRXLineResLoad(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void RXLineResLoadEMT();
	void VarFreqRXLineResLoadEMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void runDpEmtVarFreqStudy();
	
	// Circuits used for IESES18 paper
	void VarFreqRXLineResLoad_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void VarFreqRXLineResLoadEMT_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime);
	void runDpEmtVarFreqStudy_NZ_Paper();

	// Real-Time example
	void RTExample();
}
