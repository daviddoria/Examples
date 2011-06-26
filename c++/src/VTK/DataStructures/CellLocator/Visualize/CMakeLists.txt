cmake_minimum_required(VERSION 2.6)

PROJECT(VisualizeCellLocator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualizeCellLocator VisualizeCellLocator.cxx)
TARGET_LINK_LIBRARIES(VisualizeCellLocator vtkHybrid vtkWidgets)

