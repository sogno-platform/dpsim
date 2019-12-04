include(FindPackageHandleStandardArgs)

if (WIN32)
    # GSL is currently not supported for Windows
else()
    find_path(GSL_INCLUDE_DIR
        NAMES gsl_version.h
        PATH_SUFFIXES gsl
    )

    find_library(GSL_LIBRARY NAMES libgsl.so)
    find_library(GSL_CBLAS_LIBRARY NAMES libgslcblas.so)


    set(GSL_LIBRARIES
        ${GSL_LIBRARY}
        ${GSL_CBLAS_LIBRARY}
    )

   # handle the QUIETLY and REQUIRED arguments and set SUNDIALS_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(GSL DEFAULT_MSG GSL_LIBRARY GSL_INCLUDE_DIR)
    mark_as_advanced(GSL_INCLUDE_DIR)
endif()
