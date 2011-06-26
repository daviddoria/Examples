cmake_minimum_required(VERSION 2.6)

PROJECT(CombineImages)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CombineImages CombineImages.cxx)
TARGET_LINK_LIBRARIES(CombineImages vtkHybrid)


