cmake_minimum_required(VERSION 2.6)

PROJECT(PointSize)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointSize PointSize.cxx)
TARGET_LINK_LIBRARIES(PointSize vtkHybrid)
