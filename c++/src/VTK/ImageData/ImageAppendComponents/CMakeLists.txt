cmake_minimum_required(VERSION 2.6)
PROJECT(ImageAppendComponents)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageAppendComponents ImageAppendComponents.cxx)
TARGET_LINK_LIBRARIES(ImageAppendComponents vtkHybrid)

