cmake_minimum_required(VERSION 2.6)

PROJECT(ClickWorldCoordinates)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ClickWorldCoordinates ClickWorldCoordinates.cxx)
TARGET_LINK_LIBRARIES(ClickWorldCoordinates vtkHybrid)


