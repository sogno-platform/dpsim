option(CIMPP_DIR "Location of CIMpp library" ${CMAKE_CURRENT_SOURCE_DIR}/../cimpp)

if(IS_DIRECTORY ${CIMPP_DIR})
    set(CIMPP_INCLUDE_DIR ${CIMPP_DIR})
endif()

find_library(CIMPP_LIBRARY NAMES cimpp libcimpp
        HINTS ${CIMPP_DIR}/cmake-build-debug)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CIMPP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CIMpp DEFAULT_MSG
        CIMPP_LIBRARY CIMPP_INCLUDE_DIR)

mark_as_advanced(CIMPP_INCLUDE_DIR CIMPP_LIBRARY)

set(CIMPP_LIBRARIES ${CIMPP_LIBRARY})
set(CIMPP_INCLUDE_DIRS ${CIMPP_INCLUDE_DIR} ${CIMPP_DIR}/cmake-build-debug/include)
