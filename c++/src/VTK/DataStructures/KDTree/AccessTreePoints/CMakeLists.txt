cmake_minimum_required(VERSION 2.6)

PROJECT(AccessTreePoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AccessTreePoints AccessTreePoints.cxx)
TARGET_LINK_LIBRARIES(AccessTreePoints vtkHybrid)
