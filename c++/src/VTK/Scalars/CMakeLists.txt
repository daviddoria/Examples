cmake_minimum_required(VERSION 2.6)
PROJECT(Scalars)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Scalars Scalars.cxx)
TARGET_LINK_LIBRARIES(Scalars vtkHybrid)
