cmake_minimum_required(VERSION 2.6)

PROJECT(WindowedSincSmoothing)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WindowedSincSmoothing WindowedSincSmoothing.cxx)
TARGET_LINK_LIBRARIES(WindowedSincSmoothing vtkHybrid)

