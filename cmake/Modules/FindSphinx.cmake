find_program(SPHINX_EXECUTABLE
  NAMES sphinx-build
  DOC "Sphinx documentation generation tool"
  )
mark_as_advanced(SPHINX_EXECUTABLE)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  Sphinx
  "Failed to find Sphinx executable"
  SPHINX_EXECUTABLE
  )

if(SPHINX_EXECUTABLE)
  if(NOT TARGET Sphinx::sphinx)
    add_executable(Sphinx::sphinx IMPORTED GLOBAL)
    set_target_properties(Sphinx::sphinx
      PROPERTIES IMPORTED_LOCATION "${SPHINX_EXECUTABLE}"
      )
  endif()
endif()
