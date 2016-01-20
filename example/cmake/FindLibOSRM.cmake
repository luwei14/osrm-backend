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
set(LibOSRM_INCLUDE_DIRS ${LibOSRM_INCLUDE_DIR})

if ("${PC_LibOSRM_STATIC_LIBRARIES}" STREQUAL "")
  find_library(LibOSRM_STATIC_LIBRARY Names osrm.lib libosrm.a
    PATH_SUFFIXES osrm lib/osrm lib
    HINTS ${PC_LibOSRM_LIBDIR} ${PC_LibOSRM_LIBRARY_DIRS}
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local
    /usr
    /opt/local
    /opt)
  set(LibOSRM_STATIC_LIBRARIES ${LibOSRM_STATIC_LIBRARY})
else()
  set(LibOSRM_STATIC_LIBRARIES ${PC_LibOSRM_STATIC_LIBRARIES})
endif()

if ("${PC_LibOSRM_LIBRARIES}" STREQUAL "")
  find_library(LibOSRM_DYNAMIC_LIBRARY Names osrm.dynlib libosrm.so
    PATH_SUFFIXES osrm lib/osrm lib
    HINTS ${PC_LibOSRM_LIBDIR} ${PC_LibOSRM_LIBRARY_DIRS}
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local
    /usr
    /opt/local
    /opt)
  set(LibOSRM_DYNAMIC_LIBRARIES ${LibOSRM_DYNAMIC_LIBRARY})
else()
  set(LibOSRM_DYNAMIC_LIBRARIES ${PC_LibOSRM_LIBRARIES})
endif()

if ("${LibOSRM_STATIC_LIBRARIES}" STREQUAL "")
  set(LibOSRM_LIBRARIES ${LibOSRM_DYNAMIC_LIBRARIES})
else()
  set(LibOSRM_LIBRARIES ${LibOSRM_STATIC_LIBRARIES})
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBOSRM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibOSRM DEFAULT_MSG
                                LibOSRM_LIBRARIES LibOSRM_INCLUDE_DIR)
