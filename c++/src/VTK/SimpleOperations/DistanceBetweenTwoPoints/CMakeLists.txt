cmake_minimum_required(VERSION 2.6)

PROJECT(DistanceBetweenTwoPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DistanceBetweenTwoPoints DistanceBetweenTwoPoints.cpp)

TARGET_LINK_LIBRARIES(DistanceBetweenTwoPoints vtkHybrid)
