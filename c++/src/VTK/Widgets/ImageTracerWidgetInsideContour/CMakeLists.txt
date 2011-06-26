cmake_minimum_required(VERSION 2.6)

PROJECT(ImageTracerWidgetInsideContour)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageTracerWidgetInsideContour ImageTracerWidgetInsideContour.cxx)
TARGET_LINK_LIBRARIES(ImageTracerWidgetInsideContour vtkHybrid vtkWidgets)
