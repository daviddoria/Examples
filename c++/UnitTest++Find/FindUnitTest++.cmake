# set the default UNITTEST++_ROOT value
IF(NOT UNITTEST++_ROOT)
  SET(UNITTEST++_ROOT $ENV{UNITTEST++_ROOT} CACHE PATH "The UnitTest++ directory root.")
ENDIF(NOT UNITTEST++_ROOT)

FIND_LIBRARY(UNITTEST++_LIBRARY
  NAMES
    UnitTest++
    UnitTest++.vsnet2005
    UnitTest++.vsnet2008
  PATHS
    ${UNITTEST++_ROOT}/Release
    3rd-party/UnitTest++/Release
  DOC "The UnitTest++ library")

FIND_LIBRARY(UNITTEST++_LIBRARY_DEBUG
  NAMES
    UnitTest++
    UnitTest++.vsnet2005
    UnitTest++.vsnet2008
  PATHS
    ${UNITTEST++_ROOT}/Debug
    3rd-party/UnitTest++/Debug
  DOC "The UnitTest++ debug library")

IF(NOT UNITTEST++_LIBRARY_DEBUG)
  SET(UNITTEST++_LIBRARY_DEBUG ${UNITTEST++_LIBRARY})
  SET(UNITTEST++_LIBRARIES ${UNITTEST++_LIBRARY})
ELSE()
  SET(UNITTEST++_LIBRARIES
    optimized ${UNITTEST++_LIBRARY}
    debug ${UNITTEST++_LIBRARY_DEBUG}
  )
ENDIF()

FIND_PATH(UNITTEST++_INCLUDE_DIR
  NAMES UnitTest++.h
  PATHS
    ${UNITTEST++_ROOT}/src
    3rd-party/UnitTest++/src
  DOC "The UnitTest++ include files")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(UnitTest++ DEFAULT_MSG
  UNITTEST++_LIBRARY UNITTEST++_INCLUDE_DIR)