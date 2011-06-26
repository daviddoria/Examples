cmake_minimum_required(VERSION 2.6)

PROJECT(DeletePoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DeletePoints DeletePoints.cxx)
TARGET_LINK_LIBRARIES(DeletePoints vtkHybrid)
