if(NOT MAGMA_FOUND)

    include(FindPackageHandleStandardArgs)

    find_path(MAGMA_INCLUDE_DIR
        NAMES magma_v2.h magmasparse.h
        PATH_SUFFIXES "include"
        PATHS ${MAGMA_DIR}
    )
    find_library(MAGMA_LIBRARY
	    NAMES magma
        PATH_SUFFIXES "lib"
        PATHS ${MAGMA_DIR}
    )
    find_library(MAGMA_SPARSE_LIBRARY
	    NAMES magma_sparse
        PATH_SUFFIXES "lib"
        PATHS ${MAGMA_DIR}
    )


    if(MAGMA_LIBRARY AND MAGMA_SPARSE_LIBRARY)
        set(MAGMA_LIBRARIES ${MAGMA_LIBRARY} ${MAGMA_SPARSE_LIBRARY})
    endif()
    if(MAGMA_LIBRARIES AND MAGMA_INCLUDE_DIR)
        set(MAGMA_FOUND TRUE)
    else()
        set(MAGMA_FOUND FALSE)
    endif()
    find_package_handle_standard_args(MAGMA DEFAULT_MSG MAGMA_LIBRARIES MAGMA_INCLUDE_DIR)
endif(NOT MAGMA_FOUND)
