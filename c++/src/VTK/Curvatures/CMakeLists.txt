cmake_minimum_required(VERSION 2.6)

PROJECT(Curvatures)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Curvatures Curvatures.cxx)
TARGET_LINK_LIBRARIES(Curvatures vtkHybrid)
