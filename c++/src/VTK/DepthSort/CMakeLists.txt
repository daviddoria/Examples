cmake_minimum_required(VERSION 2.6)
PROJECT(DepthSort)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DepthSort DepthSort.cxx)
TARGET_LINK_LIBRARIES(DepthSort vtkHybrid)

