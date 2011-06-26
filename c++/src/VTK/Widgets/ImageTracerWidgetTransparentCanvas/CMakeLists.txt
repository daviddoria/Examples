cmake_minimum_required(VERSION 2.6)

PROJECT(ImageTracerWidgetTransparentCanvas)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageTracerWidgetTransparentCanvas ImageTracerWidgetTransparentCanvas.cxx)
TARGET_LINK_LIBRARIES(ImageTracerWidgetTransparentCanvas vtkHybrid vtkWidgets)
