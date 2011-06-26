cmake_minimum_required(VERSION 2.6)
PROJECT(ImageToStructuredPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageToStructuredPoints ImageToStructuredPoints.cxx)
TARGET_LINK_LIBRARIES(ImageToStructuredPoints vtkHybrid)

