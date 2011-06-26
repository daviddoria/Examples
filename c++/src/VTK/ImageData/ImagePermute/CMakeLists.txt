cmake_minimum_required(VERSION 2.6)
PROJECT(ImagePermute)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImagePermute ImagePermute.cxx)
TARGET_LINK_LIBRARIES(ImagePermute vtkHybrid)

