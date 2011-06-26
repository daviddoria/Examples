cmake_minimum_required(VERSION 2.6)

PROJECT(MarkPixel)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MarkPixel MarkPixel.cxx)
TARGET_LINK_LIBRARIES(MarkPixel vtkHybrid)


