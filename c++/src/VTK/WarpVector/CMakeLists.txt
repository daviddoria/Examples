cmake_minimum_required(VERSION 2.6)

PROJECT(WarpVector)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WarpVector WarpVector.cxx)
TARGET_LINK_LIBRARIES(WarpVector vtkHybrid)
