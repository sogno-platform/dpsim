/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <graphviz/cgraph.h>
#include <graphviz/gvc.h>
#include <graphviz/gvcommon.h>

#include <dpsim-models/CIM/Reader.h>
#include <dpsim-models/Definitions.h>

using namespace CPS;
using namespace CPS::CIM;

static GVC_t *gvc;
static graph_t *G;

#ifndef _WIN32
static void intr(int s) {
  /* if interrupted we try to produce a partial rendering before exiting */
  if (G)
    gvRenderJobs(gvc, G);

  /* Note that we don't call gvFinalize() so that we don't start event-driven
 	 * devices like -Tgtk or -Txlib */
  exit(gvFreeContext(gvc));
}
#endif

struct GVC_s {
  GVCOMMON_t common;

  char *config_path;
  bool config_found;

  /* gvParseArgs */
  char **input_filenames;
};

int main(int argc, char *argv[]) {
  int ret, i;

  argv[0] = (char *)"neato"; /* Default layout engine */

  gvc = gvContext();
  gvParseArgs(gvc, argc, argv);

#ifndef _WIN32
  signal(SIGUSR1, gvToggle);
  signal(SIGINT, intr);
#endif

  Reader reader("cimviz", Logger::Level::info);

  std::list<fs::path> filenames;
  for (i = 0; gvc->input_filenames[i]; i++) {
    filenames.emplace_back(gvc->input_filenames[i]);
  }

  if (i == 0)
    return 0;

  /* The remaining code is identical to GraphVizs dot.c code */
  SystemTopology sys = reader.loadCIM(50, filenames);
  Graph::Graph graph = sys.topologyGraph();

  G = graph.mGraph;

  ret = gvLayoutJobs(gvc, G); /* take layout engine from command line */
  if (ret)
    goto out;

  ret = gvRenderJobs(gvc, G);

out:
  gvFreeContext(gvc);

  return ret;
}
