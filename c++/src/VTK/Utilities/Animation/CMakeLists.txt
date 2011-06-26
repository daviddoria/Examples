cmake_minimum_required(VERSION 2.6)
PROJECT(Animation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Animation Animation.cxx)
TARGET_LINK_LIBRARIES(Animation vtkHybrid)


ADD_EXECUTABLE(Animation2 Animation2.cxx)
TARGET_LINK_LIBRARIES(Animation2 vtkHybrid)