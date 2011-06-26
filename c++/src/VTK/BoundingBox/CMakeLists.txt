cmake_minimum_required(VERSION 2.6)
PROJECT(BoundingBox)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BoundingBox BoundingBox.cxx)
TARGET_LINK_LIBRARIES(BoundingBox vtkHybrid)

