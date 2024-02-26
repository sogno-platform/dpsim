#include <dpsim/MNASolverDynInterface.h>

#include <stdio.h>
#include <string.h>

#include <cusolverSp.h>
#include <magma_types.h>
#include <magma_v2.h>
#include <magmasparse.h>

int dps_magma_init(struct dpsim_csr_matrix *matrix);
int dps_magma_decomp(struct dpsim_csr_matrix *matrix);
int dps_magma_solve(double *rhs_values, double *lhs_values);
void dps_magma_log(const char *str);
void dps_magma_cleanup(void);

struct dps_magma_data {
  int size;
  int nnz;
  int *perm_map;
  double *rhs_buffer;
  /// Solver-Handle
  magma_dopts mMagmaOpts;
  magma_queue_t mMagmaQueue;

  /// Systemmatrix
  magma_d_matrix mHostSysMat;
  magma_d_matrix mDevSysMat;

  /// RHS-Vector
  magma_d_matrix mHostRhsVec;
  magma_d_matrix mDevRhsVec;
  /// LHS-Vector
  magma_d_matrix mHostLhsVec;
  magma_d_matrix mDevLhsVec;
} data;

static const char *PLUGIN_NAME = "libdps_magma";
static struct dpsim_mna_plugin dps_magma_plugin = {
    .log =
        dps_magma_log, //a properly working dpsim will override this with the spdlog logger
    .init = dps_magma_init,
    .lu_decomp = dps_magma_decomp,
    .solve = dps_magma_solve,
    .cleanup = dps_magma_cleanup,
};

struct dpsim_mna_plugin *get_mna_plugin(const char *name) {
  if (name == NULL || strcmp(name, PLUGIN_NAME) != 0) {
    printf("error: name mismatch\n");
    return NULL;
  }
  return &dps_magma_plugin;
}

int dps_magma_init(struct dpsim_csr_matrix *matrix) {
  dps_magma_plugin.log("initialize");
  magma_init();
  magma_queue_create(0, &data.mMagmaQueue);
  data.mHostSysMat.storage_type = Magma_CSR;
  data.mDevSysMat.storage_type = Magma_CSR;
  data.mHostRhsVec.storage_type = Magma_CSR;
  data.mDevRhsVec.storage_type = Magma_CSR;
  data.mHostLhsVec.storage_type = Magma_CSR;
  data.mDevLhsVec.storage_type = Magma_CSR;
  data.rhs_buffer = (double *)malloc(sizeof(double) * matrix->row_number);
  data.perm_map = (int *)malloc(sizeof(int) * matrix->row_number);

  return dps_magma_decomp(matrix);
}

int dps_magma_decomp(struct dpsim_csr_matrix *matrix) {
  int perm_size = 0;
  data.size = matrix->row_number;
  data.nnz = matrix->nnz;
  dps_magma_plugin.log("decomp");
  cusparseMatDescr_t descr_M = 0;

  cusolverSpHandle_t mCusolverhandle;
  if (cusolverSpCreate(&mCusolverhandle) != CUSOLVER_STATUS_SUCCESS) {
    dps_magma_plugin.log("cusolverSpCreate returend an error");
    return 1;
  }
  if (cusparseCreateMatDescr(&descr_M) != CUSPARSE_STATUS_SUCCESS) {
    dps_magma_plugin.log("returend an error");
    return 1;
  }

  if (cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO) !=
      CUSPARSE_STATUS_SUCCESS) {
    dps_magma_plugin.log("cusolver returend an error");
    return 1;
  }
  if (cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL) !=
      CUSPARSE_STATUS_SUCCESS) {
    dps_magma_plugin.log("cusolver returend an error");
    return 1;
  }

  if (cusolverSpDcsrzfdHost(mCusolverhandle, matrix->row_number, matrix->nnz,
                            descr_M, matrix->values, matrix->rowIndex,
                            matrix->colIndex, data.perm_map,
                            &perm_size) != CUSOLVER_STATUS_SUCCESS) {
    dps_magma_plugin.log("cusolverSpDcsrzfdHost returend an error");
    return 1;
  }

  size_t bufferSize = 0;
  int *q = (int *)malloc(sizeof(int) * matrix->row_number);
  int *map = (int *)malloc(sizeof(int) * matrix->nnz);
  double *old_values = (double *)malloc(sizeof(double) * matrix->nnz);
  for (int i = 0; i <= matrix->row_number; ++i) {
    q[i] = i;
  }
  for (int i = 0; i <= matrix->nnz; ++i) {
    map[i] = i;
    old_values[i] = matrix->values[i];
  }

  if (cusolverSpXcsrperm_bufferSizeHost(
          mCusolverhandle, matrix->row_number, matrix->row_number, matrix->nnz,
          descr_M, matrix->rowIndex, matrix->colIndex, data.perm_map, q,
          &bufferSize) != CUSOLVER_STATUS_SUCCESS) {
    dps_magma_plugin.log("cusolverSpXcsrperm_bufferSizeHost returend an error");
    return 1;
  }

  void *buffer = malloc(bufferSize);

  if (cusolverSpXcsrpermHost(mCusolverhandle, matrix->row_number,
                             matrix->row_number, matrix->nnz, descr_M,
                             matrix->rowIndex, matrix->colIndex, data.perm_map,
                             q, map, buffer) != CUSOLVER_STATUS_SUCCESS) {
    dps_magma_plugin.log("cusolverSpXcsrpermHost returend an error");
    return 1;
  }

  for (int i = 0; i <= matrix->nnz; ++i) {
    matrix->values[i] = old_values[map[i]];
  }

  magma_dcsrset(matrix->row_number, matrix->row_number, matrix->rowIndex,
                matrix->colIndex, matrix->values, &data.mHostSysMat,
                data.mMagmaQueue);

  data.mMagmaOpts.solver_par.solver = Magma_PIDRMERGE;
  data.mMagmaOpts.solver_par.restart = 8;
  data.mMagmaOpts.solver_par.maxiter = 1000;
  data.mMagmaOpts.solver_par.rtol = 1e-10;
  data.mMagmaOpts.solver_par.maxiter = 1000;
  data.mMagmaOpts.precond_par.solver = Magma_ILU;
  data.mMagmaOpts.precond_par.levels = 0;
  data.mMagmaOpts.precond_par.trisolver = Magma_CUSOLVE;

  magma_dsolverinfo_init(&data.mMagmaOpts.solver_par,
                         &data.mMagmaOpts.precond_par, data.mMagmaQueue);

  magma_dvinit(&data.mDevRhsVec, Magma_DEV, matrix->row_number, 1, 0.0,
               data.mMagmaQueue);
  magma_dmtransfer(data.mHostSysMat, &data.mDevSysMat, Magma_CPU, Magma_DEV,
                   data.mMagmaQueue);
  magma_d_precondsetup(data.mDevSysMat, data.mDevRhsVec,
                       &data.mMagmaOpts.solver_par,
                       &data.mMagmaOpts.precond_par, data.mMagmaQueue);

  free(buffer);
  free(q);
  free(map);
  free(old_values);
  cusparseDestroyMatDescr(descr_M);
  cusolverSpDestroy(mCusolverhandle);

  return 0;
}

int dps_magma_solve(double *rhs_values, double *lhs_values) {
  int one = 0;
  dps_magma_plugin.log("solve");

  for (int i = 0; i <= data.size; ++i) {
    data.rhs_buffer[i] = rhs_values[data.perm_map[i]];
  }
  //
  //Copy right vector to device
  magma_dvset(data.size, 1, data.rhs_buffer, &data.mHostRhsVec,
              data.mMagmaQueue);
  magma_dmtransfer(data.mHostRhsVec, &data.mDevRhsVec, Magma_CPU, Magma_DEV,
                   data.mMagmaQueue);
  magma_dvinit(&data.mDevLhsVec, Magma_DEV, data.mHostRhsVec.num_rows,
               data.mHostRhsVec.num_cols, 0.0, data.mMagmaQueue);

  // Solve
  magma_d_solver(data.mDevSysMat, data.mDevRhsVec, &data.mDevLhsVec,
                 &data.mMagmaOpts, data.mMagmaQueue);

  //Copy Solution back
  magma_dmtransfer(data.mDevLhsVec, &data.mHostLhsVec, Magma_DEV, Magma_CPU,
                   data.mMagmaQueue);
  magma_dvcopy(data.mDevLhsVec, &data.size, &one, lhs_values, data.mMagmaQueue);

  //Apply inverse Permutation: TODO: This seems to be wrong, but why?
  //this->mLeftSideVector = mTransp->inverse() * this->mLeftSideVector;

  // Components' states will be updated by the post-step tasks
  return 0;
}

void dps_magma_cleanup(void) {
  dps_magma_plugin.log("cleanup");
  magma_dmfree(&data.mDevSysMat, data.mMagmaQueue);
  magma_dmfree(&data.mDevRhsVec, data.mMagmaQueue);
  magma_dmfree(&data.mDevLhsVec, data.mMagmaQueue);

  magma_queue_destroy(data.mMagmaQueue);
  magma_finalize();
  free(data.perm_map);
  free(data.rhs_buffer);
}

void dps_magma_log(const char *str) { puts(str); }
