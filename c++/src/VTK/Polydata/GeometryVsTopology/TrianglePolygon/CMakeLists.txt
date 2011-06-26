cmake_minimum_required(VERSION 2.6)

PROJECT(TrianglePolygon)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TrianglePolygon TrianglePolygon.cxx)
TARGET_LINK_LIBRARIES(TrianglePolygon vtkHybrid)
