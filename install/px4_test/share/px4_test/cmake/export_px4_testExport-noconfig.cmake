#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "px4_test::frame_transforms" for configuration ""
set_property(TARGET px4_test::frame_transforms APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(px4_test::frame_transforms PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libframe_transforms.so"
  IMPORTED_SONAME_NOCONFIG "libframe_transforms.so"
  )

list(APPEND _cmake_import_check_targets px4_test::frame_transforms )
list(APPEND _cmake_import_check_files_for_px4_test::frame_transforms "${_IMPORT_PREFIX}/lib/libframe_transforms.so" )

# Import target "px4_test::px4_test" for configuration ""
set_property(TARGET px4_test::px4_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(px4_test::px4_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/px4_test/px4_test"
  )

list(APPEND _cmake_import_check_targets px4_test::px4_test )
list(APPEND _cmake_import_check_files_for_px4_test::px4_test "${_IMPORT_PREFIX}/lib/px4_test/px4_test" )

# Import target "px4_test::gazebo_clock" for configuration ""
set_property(TARGET px4_test::gazebo_clock APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(px4_test::gazebo_clock PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/px4_test/gazebo_clock"
  )

list(APPEND _cmake_import_check_targets px4_test::gazebo_clock )
list(APPEND _cmake_import_check_files_for_px4_test::gazebo_clock "${_IMPORT_PREFIX}/lib/px4_test/gazebo_clock" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
