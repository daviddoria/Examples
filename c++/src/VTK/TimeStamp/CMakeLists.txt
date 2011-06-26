cmake_minimum_required(VERSION 2.6)
PROJECT(TimeStamp)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TimeStamp TimeStamp.cxx)
TARGET_LINK_LIBRARIES(TimeStamp vtkHybrid)

