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

find_path(GRAPHVIZ_INCLUDE_DIR
    NAMES
        graphviz/cgraph.h
    PATH_SUFFIXES
        include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Graphviz REQUIRED_VARS CGRAPH_LIBRARY GVC_LIBRARY GRAPHVIZ_INCLUDE_DIR)

mark_as_advanced(GRAPHVIZ_LIBRARY GRAPHVIZ_INCLUDE_DIR)

set(GRAPHVIZ_LIBRARIES ${CGRAPH_LIBRARY} ${GVC_LIBRARY})
set(GRAPHVIZ_INCLUDE_DIRS ${GRAPHVIZ_INCLUDE_DIR})
