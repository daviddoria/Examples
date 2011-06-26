cmake_minimum_required(VERSION 2.6)

PROJECT(BuildLocator_ClosestPoint)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BuildLocator_ClosestPoint BuildLocator_ClosestPoint.cxx)
TARGET_LINK_LIBRARIES(BuildLocator_ClosestPoint vtkHybrid)
