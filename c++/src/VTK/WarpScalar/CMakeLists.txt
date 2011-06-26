cmake_minimum_required(VERSION 2.6)

PROJECT(WarpScalar)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WarpScalar WarpScalar.cxx)
TARGET_LINK_LIBRARIES(WarpScalar vtkHybrid)
