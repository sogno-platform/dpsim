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

# Find the directory that directly contains the Graphviz headers.
#
# Typical locations:
#   Linux:   /usr/include/graphviz
#   macOS:   /opt/homebrew/include/graphviz
#   Windows: <Graphviz>/include/graphviz
#
# Some installations may place the headers directly in include/, so keep
# that as a fallback.
find_path(GRAPHVIZ_INCLUDE_DIR
	NAMES
		cgraph.h
	PATH_SUFFIXES
		include/graphviz
		graphviz
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
		#include <gvc.h>

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
		add_compile_definitions(GRAPHVIZ_RENDERDATA_USES_SIZE_T)
	else()
		message(STATUS "Graphviz gvRenderData uses unsigned int for output length")
	endif()
endif()

mark_as_advanced(
	CGRAPH_LIBRARY
	GVC_LIBRARY
	GRAPHVIZ_INCLUDE_DIR
)
