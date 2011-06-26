cmake_minimum_required(VERSION 2.6)
PROJECT(VectorText)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VectorText VectorText.cxx)
TARGET_LINK_LIBRARIES(VectorText vtkHybrid)

