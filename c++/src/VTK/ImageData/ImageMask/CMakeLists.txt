cmake_minimum_required(VERSION 2.6)
PROJECT(ImageMask)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMask ImageMask.cxx)
TARGET_LINK_LIBRARIES(ImageMask vtkHybrid)
