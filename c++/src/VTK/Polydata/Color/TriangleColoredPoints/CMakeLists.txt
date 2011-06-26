cmake_minimum_required(VERSION 2.6)

PROJECT(TriangleColoredPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TriangleColoredPoints TriangleColoredPoints.cxx)
TARGET_LINK_LIBRARIES(TriangleColoredPoints vtkHybrid)
