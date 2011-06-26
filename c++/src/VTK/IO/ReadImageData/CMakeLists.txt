cmake_minimum_required(VERSION 2.6)

PROJECT(ReadImageData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadImageData ReadImageData.cxx)
TARGET_LINK_LIBRARIES(ReadImageData vtkHybrid)
