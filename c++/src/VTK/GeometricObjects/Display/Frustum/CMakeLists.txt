cmake_minimum_required(VERSION 2.6)

PROJECT(Frustum)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DisplayFrustum DisplayFrustum.cxx)
TARGET_LINK_LIBRARIES(DisplayFrustum vtkHybrid)
