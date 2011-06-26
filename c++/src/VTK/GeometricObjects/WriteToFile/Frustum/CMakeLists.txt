cmake_minimum_required(VERSION 2.6)

PROJECT(Frustum)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Frustum Frustum.cxx)
TARGET_LINK_LIBRARIES(Frustum vtkHybrid)
