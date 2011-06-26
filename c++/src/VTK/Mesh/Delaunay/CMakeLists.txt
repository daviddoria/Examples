cmake_minimum_required(VERSION 2.6)

PROJECT(Delaunay)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Delaunay Delaunay.cxx)
TARGET_LINK_LIBRARIES(Delaunay vtkHybrid)
