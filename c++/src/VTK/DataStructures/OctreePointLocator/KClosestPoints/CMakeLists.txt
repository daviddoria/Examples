cmake_minimum_required(VERSION 2.6)

PROJECT(KClosestPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KClosestPoints KClosestPoints.cxx)
TARGET_LINK_LIBRARIES(KClosestPoints vtkHybrid)

ADD_EXECUTABLE(KClosestPointsDemo KClosestPointsDemo.cxx)
TARGET_LINK_LIBRARIES(KClosestPointsDemo vtkHybrid)
