#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "flann::flann_cpp_s" for configuration "Release"
set_property(TARGET flann::flann_cpp_s APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(flann::flann_cpp_s PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/flann_cpp_s.lib"
  )

list(APPEND _cmake_import_check_targets flann::flann_cpp_s )
list(APPEND _cmake_import_check_files_for_flann::flann_cpp_s "${_IMPORT_PREFIX}/lib/flann_cpp_s.lib" )

# Import target "flann::flann_s" for configuration "Release"
set_property(TARGET flann::flann_s APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(flann::flann_s PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/flann_s.lib"
  )

list(APPEND _cmake_import_check_targets flann::flann_s )
list(APPEND _cmake_import_check_files_for_flann::flann_s "${_IMPORT_PREFIX}/lib/flann_s.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
