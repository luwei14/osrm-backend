# - Try to find LibOSRM
# Once done this will define
#  LibOSRM_FOUND - System has LibOSRM
#  LibOSRM_INCLUDE_DIRS - The LibOSRM include directories
#  LibOSRM_LIBRARIES - The libraries needed to use LibOSRM
#  LibOSRM_DEFINITIONS - Compiler switches required for using LibOSRM

find_package(PkgConfig)
pkg_check_modules(PC_LibOSRM QUIET libosrm)
set(LibOSRM_DEFINITIONS ${PC_LibOSRM_CFLAGS_OTHER})

find_path(LibOSRM_INCLUDE_DIR osrm/osrm.hpp
  PATH_SUFFIXES osrm include/osrm include
  HINTS ${PC_LibOSRM_INCLUDEDIR} ${PC_LibOSRM_INCLUDE_DIRS}
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /opt/local
  /opt)

find_library(LibOSRM_LIBRARY Names OSRM libosrm
  PATH_SUFFIXES osrm lib/osrm lib
  HINTS ${PC_LibOSRM_LIBDIR} ${PC_LibOSRM_LIBRARY_DIRS}
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /opt/local
  /opt)

set(LibOSRM_LIBRARIES ${LibOSRM_LIBRARY})
set(LibOSRM_INCLUDE_DIRS ${LibOSRM_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBOSRM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibOSRM  DEFAULT_MSG
                                LibOSRM_LIBRARY LibOSRM_INCLUDE_DIR)
