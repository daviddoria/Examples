cmake_minimum_required(VERSION 2.6)

PROJECT(Subdivide)

#########################################3

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
ADD_EXECUTABLE(Subdivide Subdivide.cpp)
TARGET_LINK_LIBRARIES(Subdivide vtkHybrid)

