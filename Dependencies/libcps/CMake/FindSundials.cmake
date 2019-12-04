include(FindPackageHandleStandardArgs)

if (WIN32)
    message(STATIS "Sundials is currently not supported in Windows")
else()
    find_path(SUNDIALS_INCLUDE_DIR
        NAMES sundials_version.h
        PATH_SUFFIXES sundials
    )

    find_library(SUNDIALS_ARKODE_LIBRARY NAMES sundials_arkode)
    find_library(SUNDIALS_CVODE_LIBRARY  NAMES sundials_cvode)
    find_library(SUNDIALS_CVODES_LIBRARY NAMES sundials_cvodes)
    find_library(SUNDIALS_IDA_LIBRARY    NAMES sundials_ida)
    find_library(SUNDIALS_IDAS_LIBRARY   NAMES sundials_idas)
    find_library(SUNDIALS_KINSOL_LIBRARY NAMES sundials_kinsol)

    set(SUNDIALS_LIBRARIES
        ${SUNDIALS_ARKODE_LIBRARY}
        ${SUNDIALS_CVODE_LIBRARY}
        ${SUNDIALS_CVODES_LIBRARY}
        ${SUNDIALS_IDA_LIBRARY}
        ${SUNDIALS_IDAS_LIBRARY}
        ${SUNDIALS_KINSOL_LIBRARY}
    )

    # handle the QUIETLY and REQUIRED arguments and set SUNDIALS_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(Sundials DEFAULT_MSG SUNDIALS_ARKODE_LIBRARY SUNDIALS_INCLUDE_DIR)
    mark_as_advanced(SUNDIALS_INCLUDE_DIR)
endif()
