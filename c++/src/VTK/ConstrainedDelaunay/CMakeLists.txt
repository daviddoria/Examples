cmake_minimum_required(VERSION 2.6)

PROJECT(ConstrainedDelaunay)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConstrainedDelaunay ConstrainedDelaunay.cxx)
TARGET_LINK_LIBRARIES(ConstrainedDelaunay vtkHybrid)
