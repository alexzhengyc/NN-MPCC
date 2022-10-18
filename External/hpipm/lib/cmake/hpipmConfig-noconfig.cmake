#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpipm" for configuration ""
set_property(TARGET hpipm APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hpipm PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "blasfeo"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhpipm.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hpipm )
list(APPEND _IMPORT_CHECK_FILES_FOR_hpipm "${_IMPORT_PREFIX}/lib/libhpipm.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
