cmake_minimum_required(VERSION 2.6)
PROJECT(ImageNormalize)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageNormalize ImageNormalize.cxx)
TARGET_LINK_LIBRARIES(ImageNormalize vtkHybrid)

