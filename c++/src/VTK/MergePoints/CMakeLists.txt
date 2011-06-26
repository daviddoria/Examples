cmake_minimum_required(VERSION 2.6)

PROJECT(MergePoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MergePoints MergePoints.cxx)
TARGET_LINK_LIBRARIES(MergePoints vtkHybrid)
