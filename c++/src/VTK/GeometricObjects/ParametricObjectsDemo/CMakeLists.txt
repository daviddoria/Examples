cmake_minimum_required(VERSION 2.6)

PROJECT(ParametricObjectsDemo)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ParametricObjectsDemo ParametricObjectsDemo.cxx)
TARGET_LINK_LIBRARIES(ParametricObjectsDemo vtkHybrid)
