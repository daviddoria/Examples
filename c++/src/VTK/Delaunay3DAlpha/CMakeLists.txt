cmake_minimum_required(VERSION 2.6)

PROJECT(Delaunay3DAlpha)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Delaunay3DAlpha Delaunay3DAlpha.cxx)
TARGET_LINK_LIBRARIES(Delaunay3DAlpha vtkHybrid)
