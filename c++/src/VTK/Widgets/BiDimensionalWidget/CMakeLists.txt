cmake_minimum_required(VERSION 2.6)

PROJECT(BiDimensionalWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BiDimensionalWidget BiDimensionalWidget.cxx)
TARGET_LINK_LIBRARIES(BiDimensionalWidget vtkHybrid vtkWidgets)
