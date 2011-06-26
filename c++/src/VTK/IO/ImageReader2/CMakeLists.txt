cmake_minimum_required(VERSION 2.6)

PROJECT(ImageReader2)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageReader2 ImageReader2.cxx)
TARGET_LINK_LIBRARIES(ImageReader2 vtkHybrid vtkInfovis)

