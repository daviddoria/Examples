cmake_minimum_required(VERSION 2.6)

PROJECT(ColoredSphere)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColoredSphere ColoredSphere.cxx)

TARGET_LINK_LIBRARIES(ColoredSphere vtkHybrid)
