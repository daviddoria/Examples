cmake_minimum_required(VERSION 2.6)

PROJECT(Dimensions)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Dimensions Dimensions.cxx)
TARGET_LINK_LIBRARIES(Dimensions vtkHybrid)

