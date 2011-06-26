cmake_minimum_required(VERSION 2.6)

PROJECT(BuildLocatorFromPoints_ClosestPoint)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BuildLocatorFromPoints_ClosestPoint BuildLocatorFromPoints_ClosestPoint.cxx)
TARGET_LINK_LIBRARIES(BuildLocatorFromPoints_ClosestPoint vtkHybrid)
