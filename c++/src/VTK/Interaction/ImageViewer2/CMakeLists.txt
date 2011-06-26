cmake_minimum_required(VERSION 2.6)

PROJECT(ImageViewer2)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageViewer2 ImageViewer2.cxx)
TARGET_LINK_LIBRARIES(ImageViewer2 vtkHybrid)

