set(FILESYSTEM_LIBRARY stdc++fs)

set(FILESYSTEM_LIBRARIES ${FILESYSTEM_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Filesystem REQUIRED_VARS FILESYSTEM_LIBRARY)

mark_as_advanced(FILESYSTEM_LIBRARY)

add_library(filesystem INTERFACE)
target_link_libraries(filesystem INTERFACE ${FILESYSTEM_LIBRARY})
