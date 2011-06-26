cmake_minimum_required(VERSION 2.6)
PROJECT(IterateImageData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(IterateImageData IterateImageData.cxx)
TARGET_LINK_LIBRARIES(IterateImageData vtkHybrid)