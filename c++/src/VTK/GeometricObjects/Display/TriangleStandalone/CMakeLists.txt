cmake_minimum_required(VERSION 2.6)

PROJECT(TriangleStandalone)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TriangleStandalone TriangleStandalone.cxx)
TARGET_LINK_LIBRARIES(TriangleStandalone vtkHybrid)
