cmake_minimum_required(VERSION 2.6)

PROJECT(PointsWithinRadius)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointsWithinRadius PointsWithinRadius.cxx)
TARGET_LINK_LIBRARIES(PointsWithinRadius vtkHybrid)

