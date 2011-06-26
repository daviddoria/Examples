cmake_minimum_required(VERSION 2.6)

PROJECT(ImageGridSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageGridSource ImageGridSource.cxx)
TARGET_LINK_LIBRARIES(ImageGridSource vtkHybrid)
