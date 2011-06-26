cmake_minimum_required(VERSION 2.6)
PROJECT(PointsProjectedHull)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointsProjectedHull PointsProjectedHull.cxx)
TARGET_LINK_LIBRARIES(PointsProjectedHull vtkHybrid)

