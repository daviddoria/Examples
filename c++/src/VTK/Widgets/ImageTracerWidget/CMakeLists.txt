cmake_minimum_required(VERSION 2.6)

PROJECT(ImageTracerWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageTracerWidget ImageTracerWidget.cxx)
TARGET_LINK_LIBRARIES(ImageTracerWidget vtkHybrid vtkWidgets)
