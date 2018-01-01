find_package(Sphinx)

if(SPHINX_EXECUTABLE)

    if(NOT DEFINED SPHINX_THEME)
        set(SPHINX_THEME alabaster)
    endif()

    if(NOT DEFINED SPHINX_THEME_DIR)
      set(SPHINX_THEME_DIR)
    endif()

    # configured documentation tools and intermediate build results
    set(BINARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_build")

    # Sphinx cache with pickled ReST documents
    set(SPHINX_CACHE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_doctrees")

    # HTML output directory
    set(SPHINX_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/html")

    set(SPHINX_TITLE "${PROJECT_NAME} Documentation")

    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/conf.py.in"
      "${BINARY_BUILD_DIR}/conf.py"
      @ONLY
    )

    # We add the path of the freshly build dpsim.so module to Python path here
    add_custom_target(docs
      ${CMAKE_COMMAND} -E env PYTHONPATH=$<TARGET_FILE_DIR:dpsim>:$ENV{PYTHONPATH}
      ${SPHINX_EXECUTABLE}
          -q -b html
          -c "${BINARY_BUILD_DIR}"
          -d "${SPHINX_CACHE_DIR}"
          "${CMAKE_CURRENT_SOURCE_DIR}"
          "${SPHINX_HTML_DIR}"
      COMMENT "Building HTML documentation with Sphinx"
      DEPENDS link_dpsim)
endif()

# add a target to generate API documentation with Doxygen
find_package(Doxygen)

if(DOXYGEN_FOUND)

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

  # HTML output directory
  set(DOXYGEN_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/Cxx")
  file(MAKE_DIRECTORY ${DOXYGEN_HTML_DIR})

  add_custom_target(docs_cxx
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )

endif(DOXYGEN_FOUND)
