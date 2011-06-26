cmake_minimum_required(VERSION 2.6)

PROJECT(FindPointsWithinRadius)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FindPointsWithinRadius FindPointsWithinRadius.cxx)
TARGET_LINK_LIBRARIES(FindPointsWithinRadius vtkHybrid)
