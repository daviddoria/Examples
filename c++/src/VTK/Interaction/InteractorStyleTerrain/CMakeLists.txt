cmake_minimum_required(VERSION 2.6)

PROJECT(InteractorStyleTerrain)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InteractorStyleTerrain InteractorStyleTerrain.cxx)
TARGET_LINK_LIBRARIES(InteractorStyleTerrain vtkHybrid)
