# - Try to find Eigen2 lib
#
# This module supports requiring a minimum version, e.g. you can do
#   find_package(Eigen2 2.0.3)
# to require version 2.0.3 to newer of Eigen2.
#
# Once done this will define
#
#  EIGEN2_FOUND - system has eigen lib with correct version
#  EIGEN2_INCLUDE_DIR - the eigen include directory
#  EIGEN2_VERSION - eigen version

# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Copyright (c) 2008, 2009 Gael Guennebaud, <g.gael@free.fr>
# Redistribution and use is allowed according to the terms of the BSD license.

if(NOT Eigen2_FIND_VERSION)
  set(Eigen2_FIND_VERSION_MAJOR 2)
  set(Eigen2_FIND_VERSION_MINOR 0)
  set(Eigen2_FIND_VERSION_PATCH 0)

  set(Eigen2_FIND_VERSION "${Eigen2_FIND_VERSION_MAJOR}.${Eigen2_FIND_VERSION_MINOR}.${Eigen2_FIND_VERSION_PATCH}")
endif(NOT Eigen2_FIND_VERSION)

macro(_eigen2_get_version)
  file(READ "${EIGEN2_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen2_version_header LIMIT 5000 OFFSET 1000)

  string(REGEX MATCH "define *EIGEN_WORLD_VERSION ([0-9]*)" _eigen2_world_version_match "${_eigen2_version_header}")
  set(EIGEN2_WORLD_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define *EIGEN_MAJOR_VERSION ([0-9]*)" _eigen2_major_version_match "${_eigen2_version_header}")
  set(EIGEN2_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define *EIGEN_MINOR_VERSION ([0-9]*)" _eigen2_minor_version_match "${_eigen2_version_header}")
  set(EIGEN2_MINOR_VERSION "${CMAKE_MATCH_1}")

  set(EIGEN2_VERSION ${EIGEN2_WORLD_VERSION}.${EIGEN2_MAJOR_VERSION}.${EIGEN2_MINOR_VERSION})
endmacro(_eigen2_get_version)

find_path(EIGEN2_INCLUDE_DIR NAMES Eigen/Core
     PATHS
     ${INCLUDE_INSTALL_DIR}
     ${KDE4_INCLUDE_DIR}
     PATH_SUFFIXES eigen2
   )

if(EIGEN2_INCLUDE_DIR)
  _eigen2_get_version()
endif(EIGEN2_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen2 REQUIRED_VARS EIGEN2_INCLUDE_DIR
                                         VERSION_VAR EIGEN2_VERSION)

mark_as_advanced(EIGEN2_INCLUDE_DIR)


