cmake_minimum_required(VERSION 2.6)

PROJECT(ArrayWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArrayWriter ArrayWriter.cxx)
TARGET_LINK_LIBRARIES(ArrayWriter vtkHybrid)
