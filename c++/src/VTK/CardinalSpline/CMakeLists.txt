cmake_minimum_required(VERSION 2.6)
PROJECT(CardinalSpline)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CardinalSpline CardinalSpline.cxx)
TARGET_LINK_LIBRARIES(CardinalSpline vtkHybrid)

