#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "flann::flann_cpp_s" for configuration "Debug"
set_property(TARGET flann::flann_cpp_s APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(flann::flann_cpp_s PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/flann_cpp_s-gd.lib"
  )

list(APPEND _cmake_import_check_targets flann::flann_cpp_s )
list(APPEND _cmake_import_check_files_for_flann::flann_cpp_s "${_IMPORT_PREFIX}/lib/flann_cpp_s-gd.lib" )

# Import target "flann::flann_s" for configuration "Debug"
set_property(TARGET flann::flann_s APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(flann::flann_s PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/flann_s-gd.lib"
  )

list(APPEND _cmake_import_check_targets flann::flann_s )
list(APPEND _cmake_import_check_files_for_flann::flann_s "${_IMPORT_PREFIX}/lib/flann_s-gd.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
