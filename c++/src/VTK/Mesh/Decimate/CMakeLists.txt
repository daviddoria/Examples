cmake_minimum_required(VERSION 2.6)

PROJECT(Decimate)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Decimate Decimate.cxx)
TARGET_LINK_LIBRARIES(Decimate vtkHybrid)

ADD_EXECUTABLE(TriangulateDecimate TriangulateDecimate.cpp)
TARGET_LINK_LIBRARIES(TriangulateDecimate vtkHybrid)
