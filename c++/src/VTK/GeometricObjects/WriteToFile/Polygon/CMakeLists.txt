cmake_minimum_required(VERSION 2.6)

PROJECT(Polygon)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Polygon Polygon.cxx)

TARGET_LINK_LIBRARIES(Polygon vtkHybrid)
