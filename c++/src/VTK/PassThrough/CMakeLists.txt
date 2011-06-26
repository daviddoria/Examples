cmake_minimum_required(VERSION 2.6)

PROJECT(PassThrough)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PassThrough PassThrough.cxx)
TARGET_LINK_LIBRARIES(PassThrough vtkHybrid vtkInfovis)
