cmake_minimum_required(VERSION 2.6)

PROJECT(PointsInsidePyramid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointsInsidePyramid PointsInsidePyramid.cxx)
TARGET_LINK_LIBRARIES(PointsInsidePyramid vtkHybrid)

ADD_EXECUTABLE(PointsInsideSphere PointsInsideSphere.cxx)
TARGET_LINK_LIBRARIES(PointsInsideSphere vtkHybrid)
