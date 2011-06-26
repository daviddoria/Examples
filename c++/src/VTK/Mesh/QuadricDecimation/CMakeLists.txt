cmake_minimum_required(VERSION 2.6)

PROJECT(QuadricDecimation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(QuadricDecimation QuadricDecimation.cxx)
TARGET_LINK_LIBRARIES(QuadricDecimation vtkHybrid)
