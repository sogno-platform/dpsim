find_library(CGRAPH_LIBRARY
	NAMES
		cgraph
	PATH_SUFFIXES
		lib
)

find_library(GVC_LIBRARY
	NAMES
		gvc
	PATH_SUFFIXES
		lib
)

# Find the directory that contains the graphviz/ header directory, so
# callers use the prefixed #include <graphviz/cgraph.h> form and this
# directory does not leak unprefixed, generically-named Graphviz headers
# (types.h, const.h, graph.h, color.h, ...) onto every consumer's include
# path. Linux and Homebrew both install to <prefix>/include/graphviz/cgraph.h.
find_path(GRAPHVIZ_INCLUDE_DIR
	NAMES
		graphviz/cgraph.h
	PATH_SUFFIXES
		include
)

set(GRAPHVIZ_LIBRARIES
	${CGRAPH_LIBRARY}
	${GVC_LIBRARY}
)

set(GRAPHVIZ_INCLUDE_DIRS
	${GRAPHVIZ_INCLUDE_DIR}
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
	Graphviz
	REQUIRED_VARS
		CGRAPH_LIBRARY
		GVC_LIBRARY
		GRAPHVIZ_INCLUDE_DIR
)

# Detect the gvRenderData() API.
#
# Older Graphviz versions use:
#   unsigned int *length
#
# Graphviz >= 13 uses:
#   size_t *length
#
# Detecting the actual function signature instead of checking the Graphviz
# version keeps this compatible across Linux, macOS and Windows.
if(Graphviz_FOUND)
	include(CheckCXXSourceCompiles)

	set(_GRAPHVIZ_SAVED_REQUIRED_INCLUDES "${CMAKE_REQUIRED_INCLUDES}")
	set(_GRAPHVIZ_SAVED_REQUIRED_LIBRARIES "${CMAKE_REQUIRED_LIBRARIES}")

	set(CMAKE_REQUIRED_INCLUDES
		${GRAPHVIZ_INCLUDE_DIR}
	)

	set(CMAKE_REQUIRED_LIBRARIES
		${GVC_LIBRARY}
		${CGRAPH_LIBRARY}
	)

	check_cxx_source_compiles(
		"
		#include <cstddef>
		#include <graphviz/gvc.h>

		int main()
		{
			GVC_t *gvc = nullptr;
			graph_t *graph = nullptr;
			char *data = nullptr;
			size_t length = 0;

			gvRenderData(gvc, graph, \"svg\", &data, &length);

			return 0;
		}
		"
		GRAPHVIZ_RENDERDATA_USES_SIZE_T
	)

	set(CMAKE_REQUIRED_INCLUDES "${_GRAPHVIZ_SAVED_REQUIRED_INCLUDES}")
	set(CMAKE_REQUIRED_LIBRARIES "${_GRAPHVIZ_SAVED_REQUIRED_LIBRARIES}")

	unset(_GRAPHVIZ_SAVED_REQUIRED_INCLUDES)
	unset(_GRAPHVIZ_SAVED_REQUIRED_LIBRARIES)

	if(GRAPHVIZ_RENDERDATA_USES_SIZE_T)
		message(STATUS "Graphviz gvRenderData uses size_t for output length")
	else()
		message(STATUS "Graphviz gvRenderData uses unsigned int for output length")
	endif()
endif()

mark_as_advanced(
	CGRAPH_LIBRARY
	GVC_LIBRARY
	GRAPHVIZ_INCLUDE_DIR
)
