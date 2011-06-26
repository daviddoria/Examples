cmake_minimum_required(VERSION 2.6)

PROJECT(Trackball)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Trackball Trackball.cxx)
TARGET_LINK_LIBRARIES(Trackball vtkHybrid)
