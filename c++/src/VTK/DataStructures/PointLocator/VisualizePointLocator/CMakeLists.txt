cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizePointLocator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizePointLocator VisualizePointLocator.cxx)
TARGET_LINK_LIBRARIES(VisualizePointLocator vtkHybrid vtkWidgets)

