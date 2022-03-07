/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once


namespace DPsimPlugin {

    class MnaSolverDynInterface {
	private:
		void (*log)(const char *);
	public:
		virtual void set_logger(void (*logger)(const char *)) = 0;
		virtual int initialize(
			int row_number,
			int nnz,
			double *csrValues,
			int *csrRowPtr,
			int *csrColInd) = 0;
		virtual int solve(
			double *rhs_values,
			double *lhs_values) = 0;
		virtual void cleanup() = 0;
	};
}

extern "C" DPsimPlugin::MnaSolverDynInterface* getMNASolverPlugin(const char *name);
