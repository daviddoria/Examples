cmake_minimum_required(VERSION 2.6)

PROJECT(PointNormals)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointNormals PointNormals.cxx)
TARGET_LINK_LIBRARIES(PointNormals vtkHybrid)

