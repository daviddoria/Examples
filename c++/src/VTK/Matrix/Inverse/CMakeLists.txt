cmake_minimum_required(VERSION 2.6)
PROJECT(Inverse)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Inverse Inverse.cxx)
TARGET_LINK_LIBRARIES(Inverse vtkHybrid)

