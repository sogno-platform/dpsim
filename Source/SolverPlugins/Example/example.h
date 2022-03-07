/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#define PLUGIN_NAME "plugin.so"

class MnaSolverPluginExample : public DPsimPlugin::MnaSolverDynInterface {
private:
    void (*log)(const char *);
public:
    MnaSolverPluginExample();
    ~MnaSolverPluginExample();

    void set_logger(void (*logger)(const char *)) override;
    int initialize(
        int row_number,
        int nnz,
        double *csrValues,
        int *csrRowPtr,
        int *csrColInd) override;
    int solve(
        double *rhs_values,
        double *lhs_values) override;
    virtual void cleanup() override;
};
