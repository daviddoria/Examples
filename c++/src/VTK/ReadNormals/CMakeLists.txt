cmake_minimum_required(VERSION 2.6)
PROJECT(ReadNormals)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadNormals ReadNormals.cpp)
TARGET_LINK_LIBRARIES(ReadNormals  vtkHybrid)

