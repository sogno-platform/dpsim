/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#ifndef _MNA_SOLVER_DYN_INTERFACE_H_
#define _MNA_SOLVER_DYN_INTERFACE_H_

struct dpsim_csr_matrix {
  double *values; //size: nnz
  int *rowIndex;  //size: nnz
  int *colIndex;  //size: row_numer+1
  int row_number; //number of rows of the matrix
  int nnz;        //number of non-zero elements in matrix
};

struct dpsim_mna_plugin {
  void (*log)(const char *);
  int (*init)(struct dpsim_csr_matrix *);
  int (*lu_decomp)(struct dpsim_csr_matrix *);
  int (*solve)(double *, double *);
  void (*cleanup)(void);
};

extern struct dpsim_mna_plugin *get_mna_plugin(const char *name);

#endif
