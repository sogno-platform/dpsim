#include <dpsim/MNASolverDynInterface.h>

#include <stdio.h>
#include <string.h>

int example_init(struct dpsim_csr_matrix *matrix);
int example_decomp(struct dpsim_csr_matrix *matrix);
int example_solve(double *rhs_values, double *lhs_values);
void example_log(const char *str);
void example_cleanup(void);

static const char *PLUGIN_NAME = "plugin.so";
static struct dpsim_mna_plugin example_plugin = {
    .log =
        example_log, //a properly working dpsim will override this with the spdlog logger
    .init = example_init,
    .lu_decomp = example_decomp,
    .solve = example_solve,
    .cleanup = example_cleanup,
};

struct dpsim_mna_plugin *get_mna_plugin(const char *name) {
  if (name == NULL || strcmp(name, PLUGIN_NAME) != 0) {
    printf("error: name mismatch\n");
    return NULL;
  }
  return &example_plugin;
}

int example_init(struct dpsim_csr_matrix *matrix) {
  example_plugin.log("initialize");
  return 0;
}

int example_decomp(struct dpsim_csr_matrix *matrix) {
  example_plugin.log("decomp");
  return 0;
}

int example_solve(double *rhs_values, double *lhs_values) {
  example_plugin.log("solve");
  return 0;
}

void example_cleanup(void) { example_plugin.log("cleanup"); }

void example_log(const char *str) { puts(str); }
