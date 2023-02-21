#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "package1::package1" for configuration "RelWithDebInfo"
set_property(TARGET package1::package1 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(package1::package1 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libpackage1.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libpackage1.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS package1::package1 )
list(APPEND _IMPORT_CHECK_FILES_FOR_package1::package1 "${_IMPORT_PREFIX}/lib/libpackage1.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
