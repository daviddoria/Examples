cmake_minimum_required(VERSION 2.6)

PROJECT(TriangleStrip)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TriangleStrip TriangleStrip.cxx)

TARGET_LINK_LIBRARIES(TriangleStrip vtkHybrid)
