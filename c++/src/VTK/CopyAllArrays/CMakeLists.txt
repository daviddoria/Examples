cmake_minimum_required(VERSION 2.6)

PROJECT(CopyAllArrays)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CopyAllArrays CopyAllArrays.cxx)
TARGET_LINK_LIBRARIES(CopyAllArrays vtkHybrid)

