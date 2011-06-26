cmake_minimum_required(VERSION 2.6)

PROJECT(ImageMapToColors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMapToColors ImageMapToColors.cxx)
TARGET_LINK_LIBRARIES(ImageMapToColors vtkHybrid)
