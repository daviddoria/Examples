cmake_minimum_required(VERSION 2.6)

PROJECT(ClosedSurface)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ClosedSurface ClosedSurface.cxx)

TARGET_LINK_LIBRARIES(ClosedSurface vtkHybrid)
