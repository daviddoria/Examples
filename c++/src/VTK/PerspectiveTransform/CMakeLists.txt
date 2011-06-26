cmake_minimum_required(VERSION 2.6)

PROJECT(PerspectiveTransform)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PerspectiveTransform PerspectiveTransform.cxx)
TARGET_LINK_LIBRARIES(PerspectiveTransform vtkHybrid)

