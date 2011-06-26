cmake_minimum_required(VERSION 2.6)

PROJECT(ImageTracerWidgetNonPlanar)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageTracerWidgetNonPlanar ImageTracerWidgetNonPlanar.cxx)
TARGET_LINK_LIBRARIES(ImageTracerWidgetNonPlanar vtkHybrid vtkWidgets)
