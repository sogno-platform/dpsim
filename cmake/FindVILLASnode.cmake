find_path(VILLASNODE_INCLUDE_DIR
	NAMES villas/shmem.h
)

find_library(VILLASNODE_LIBRARY
	NAMES villas
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set VILLASNODE_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(VILLASnode DEFAULT_MSG
        VILLASNODE_LIBRARY VILLASNODE_INCLUDE_DIR)

mark_as_advanced(VILLASNODE_INCLUDE_DIR VILLASNODE_LIBRARY)

set(VILLASNODE_LIBRARIES ${VILLASNODE_LIBRARY})
set(VILLASNODE_INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIR})
