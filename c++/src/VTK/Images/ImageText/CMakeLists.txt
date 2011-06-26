cmake_minimum_required(VERSION 2.6)

PROJECT(ImageText)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageText ImageText.cxx)
TARGET_LINK_LIBRARIES(ImageText vtkHybrid)


