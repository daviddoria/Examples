cmake_minimum_required(VERSION 2.6)

PROJECT(ConnectedComponents)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConnectedComponents ConnectedComponents.cxx)
TARGET_LINK_LIBRARIES(ConnectedComponents vtkHybrid vtkInfovis)

