cmake_minimum_required(VERSION 2.6)

PROJECT(ConvexHull)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConvexHull ConvexHull.cxx)
TARGET_LINK_LIBRARIES(ConvexHull vtkHybrid)
