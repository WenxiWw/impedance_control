#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cntr_joint_imp::cntr_joint_imp" for configuration "Release"
set_property(TARGET cntr_joint_imp::cntr_joint_imp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cntr_joint_imp::cntr_joint_imp PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/panda_adm/anselm/simulink_pipeline/install/cntr_joint_imp/libcntr_joint_imp.so"
  IMPORTED_SONAME_RELEASE "libcntr_joint_imp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cntr_joint_imp::cntr_joint_imp )
list(APPEND _IMPORT_CHECK_FILES_FOR_cntr_joint_imp::cntr_joint_imp "/home/panda_adm/anselm/simulink_pipeline/install/cntr_joint_imp/libcntr_joint_imp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
