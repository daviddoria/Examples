cmake_minimum_required(VERSION 2.6)
PROJECT(FloatingPointExceptions)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FloatingPointExceptions FloatingPointExceptions.cxx)
TARGET_LINK_LIBRARIES(FloatingPointExceptions vtkHybrid)

