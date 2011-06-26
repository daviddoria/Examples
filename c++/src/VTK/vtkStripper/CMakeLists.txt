cmake_minimum_required(VERSION 2.6)
PROJECT(Stripper)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Stripper Stripper.cxx)
TARGET_LINK_LIBRARIES(Stripper vtkHybrid)

