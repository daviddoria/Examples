cmake_minimum_required(VERSION 2.6)

PROJECT(ConstructTree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConstructTree ConstructTree.cxx)
TARGET_LINK_LIBRARIES(ConstructTree vtkHybrid)
