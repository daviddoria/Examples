cmake_minimum_required(VERSION 2.6)
PROJECT(ImageIslandRemoval2D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageIslandRemoval2D ImageIslandRemoval2D.cxx)
TARGET_LINK_LIBRARIES(ImageIslandRemoval2D vtkHybrid)

