# - Try to find the QtMobility libraries
#
# This module will search for the QtMobility libraries.
#
# It supports both a minimum version and searching for individual
# components. For the minimum version, use
# find_package(QtMobility 1.2.0). For specific components, use
# find_package(QtMobility COMPONENTS ...). See below for a list of known
# components.
#
# Once done this will define
#  QTMOBILITY_FOUND - QtMobility and all specified components were found.
#  QTMOBILITY_INCLUDE_DIR - Include directory for global QtMobility files.
#  QTMOBILITY_INCLUDE_DIRS - All found QtMobility components' include dirs.
#  QTMOBILITY_LIBRARIES - All found QtMobility components' libraries.
#  QTMOBILITY_VERSION - The version of QtMobility that was found.
#
# For each found component the following will be defined:
#  QTMOBILITY_{COMPONENT}_INCLUDE_DIR - The include directory for the component.
#  QTMOBILITY_{COMPONENT}_LIBRARY - The location of the library for the component.
#
# Note that searching for components will be limited to the specified components
# if the components option is used.
#
# Copyright (c) 2011 Arjen Hiemstra <ahiemstra@heimr.nl>
# Redistribution and use is allowed according to the terms of the BSD license.

set(QTMOBILITY_COMPONENTS
    Bearer
    Connectivity
    Contacts
    Feedback
    Gallery
    Location
    Messaging
    MultimediaKit
    Organizer
    PublishSubscribe
    Sensors
    ServiceFramework
    SystemInfo
    Versit
)

if (QtMobility_FIND_COMPONENTS)
  foreach (component ${QtMobility_FIND_COMPONENTS})
    string(TOUPPER ${component} _COMPONENT)
    set(QTMOBILITY_USE_${_COMPONENT} 1)
  endforeach (component)
endif (QtMobility_FIND_COMPONENTS)

find_path(QTMOBILITY_INCLUDE_DIR qmobilityglobal.h PATH_SUFFIXES QtMobility)

#Find the mobility version
if(QTMOBILITY_INCLUDE_DIR)
    file(READ "${QTMOBILITY_INCLUDE_DIR}/qmobilityglobal.h" _qtmobility_global_header LIMIT 2000)
    string(REGEX MATCH "#define QTM_VERSION_STR \"([0-9.]*)\"" _qtmobility_version_match  "${_qtmobility_global_header}")
    set(QTMOBILITY_VERSION "${CMAKE_MATCH_1}")
    message(STATUS "QtMobility Version ${QTMOBILITY_VERSION} detected")
endif(QTMOBILITY_INCLUDE_DIR)

set(QTMOBILITY_VARIABLES "QTMOBILITY_INCLUDE_DIR")
set(QTMOBILITY_INCLUDE_DIRS ${QTMOBILITY_INCLUDE_DIR})

#A list of files to find for specific components
set(QTMOBILITY_FIND_FILES
    QNetworkConfiguration #Bearer
    QBluetoothSocket #Connectivity
    QContact #Contacts
    QFeedbackInterface #Feedback
    QAbstractGallery #Gallery
    QLandmark #Location
    QMessage #Messaging
    QMediaPlayer #MultimediaKit
    QOrganizerItem #Organizer
    QValueSpace #PublishSubscribe
    QSensor #Sensors
    QService #ServiceFramework
    QSystemInfo #SystemInfo
    QVersitDocument #Versit
)

list(LENGTH QTMOBILITY_COMPONENTS _component_count)
math(EXPR _component_count "${_component_count} - 1")
foreach (index RANGE ${_component_count})
    list(GET QTMOBILITY_COMPONENTS ${index} component)
    list(GET QTMOBILITY_FIND_FILES ${index} file)
    string(TOUPPER ${component} _COMPONENT)
    if (NOT QtMobility_FIND_COMPONENTS OR QTMOBILITY_USE_${_COMPONENT})
        message(STATUS "Looking for QtMobility Component \"${component}\"")
        find_path(QTMOBILITY_${_COMPONENT}_INCLUDE_DIR ${file} PATH_SUFFIXES Qt${component})
        find_library(QTMOBILITY_${_COMPONENT}_LIBRARY NAMES Qt${component})
        list(APPEND QTMOBILITY_VARIABLES "QTMOBILITY_${_COMPONENT}_INCLUDE_DIR" "QTMOBILITY_${_COMPONENT}_LIBRARY")
        list(APPEND QTMOBILITY_INCLUDE_DIRS ${QTMOBILITY_${_COMPONENT}_INCLUDE_DIR})
        list(APPEND QTMOBILITY_LIBRARIES ${QTMOBILITY_${_COMPONENT}_LIBRARY})
    endif (NOT QtMobility_FIND_COMPONENTS OR QTMOBILITY_USE_${_COMPONENT})
endforeach (index)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QtMobility REQUIRED_VARS ${QTMOBILITY_VARIABLES} VERSION_VAR QTMOBILITY_VERSION)

mark_as_advanced(${QTMOBILITY_VARIABLES})
