cmake_minimum_required(VERSION 2.6)
PROJECT(FreeTypeUtilities)

include_directories(/usr/include/freetype2/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FreeTypeUtilities FreeTypeUtilities.cxx)
TARGET_LINK_LIBRARIES(FreeTypeUtilities ${VTK_LIBRARIES})
