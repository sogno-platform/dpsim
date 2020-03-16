/** Render CPS::SystemTopology via Graphviz
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <graphviz/gvc.h>
#include <graphviz/cgraph.h>
#include <graphviz/gvcommon.h>

#include <cps/Definitions.h>
#include <cps/CIM/Reader.h>

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
	boolean config_found;

	/* gvParseArgs */
	char **input_filenames;
};

int main(int argc, char *argv[]) {
	int ret, i;

	argv[0] = (char *) "neato"; /* Default layout engine */

	gvc = gvContext();
	gvParseArgs(gvc, argc, argv);

#ifndef _WIN32
	signal(SIGUSR1, gvToggle);
	signal(SIGINT, intr);
#endif

	Reader reader("cimviz", Logger::Level::info);

	std::list<std::experimental::filesystem::path> filenames;
	for (i = 0; gvc->input_filenames[i]; i++) {
		filenames.emplace_back(gvc->input_filenames[i]);
	}

	if (i == 0)
		return 0;

	/* The remaining code is identical to GraphVizs dot.c code */
	SystemTopology sys = reader.loadCIM(50, filenames);
	Graph::Graph graph = sys.topologyGraph();

	G = graph.mGraph;

	ret = gvLayoutJobs(gvc, G);  /* take layout engine from command line */
	if (ret)
		goto out;

	ret = gvRenderJobs(gvc, G);

out:	gvFreeContext(gvc);

	return ret;
}
