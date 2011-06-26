cmake_minimum_required(VERSION 2.6)

PROJECT(SegFault)

#########################################3

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################

ADD_EXECUTABLE(SegFault SegFault.cpp)

TARGET_LINK_LIBRARIES(SegFault vtkHybrid)

