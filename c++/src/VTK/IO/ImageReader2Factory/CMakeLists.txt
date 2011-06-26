cmake_minimum_required(VERSION 2.6)

PROJECT(ImageReader2Factory)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageReader2Factory ImageReader2Factory.cxx)
TARGET_LINK_LIBRARIES(ImageReader2Factory vtkHybrid)

