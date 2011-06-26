cmake_minimum_required(VERSION 2.6)

PROJECT(BorderWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BorderWidget BorderWidget.cxx)
TARGET_LINK_LIBRARIES(BorderWidget vtkHybrid vtkWidgets)
