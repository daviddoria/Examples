cmake_minimum_required(VERSION 2.6)

PROJECT(PolyDataGetPoint)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PolyDataGetPoint PolyDataGetPoint.cxx)
TARGET_LINK_LIBRARIES(PolyDataGetPoint vtkHybrid)
