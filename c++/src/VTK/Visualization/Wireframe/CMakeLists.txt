cmake_minimum_required(VERSION 2.6)

PROJECT(Wireframe)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Wireframe Wireframe.cxx)
TARGET_LINK_LIBRARIES(Wireframe vtkHybrid)

