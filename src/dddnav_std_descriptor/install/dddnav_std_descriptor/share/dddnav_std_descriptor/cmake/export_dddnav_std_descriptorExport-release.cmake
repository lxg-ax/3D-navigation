#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dddnav_std_descriptor::dddnav_std_descriptor" for configuration "Release"
set_property(TARGET dddnav_std_descriptor::dddnav_std_descriptor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dddnav_std_descriptor::dddnav_std_descriptor PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdddnav_std_descriptor.so"
  IMPORTED_SONAME_RELEASE "libdddnav_std_descriptor.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dddnav_std_descriptor::dddnav_std_descriptor )
list(APPEND _IMPORT_CHECK_FILES_FOR_dddnav_std_descriptor::dddnav_std_descriptor "${_IMPORT_PREFIX}/lib/libdddnav_std_descriptor.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
