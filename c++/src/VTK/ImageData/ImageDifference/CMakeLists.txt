cmake_minimum_required(VERSION 2.6)
PROJECT(ImageDifference)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageDifference ImageDifference.cxx)
TARGET_LINK_LIBRARIES(ImageDifference vtkHybrid)

