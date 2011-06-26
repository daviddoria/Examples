cmake_minimum_required(VERSION 2.6)

PROJECT(CreateTree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CreateTree CreateTree.cxx)
TARGET_LINK_LIBRARIES(CreateTree vtkHybrid)
