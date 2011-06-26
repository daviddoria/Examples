cmake_minimum_required(VERSION 2.6)

PROJECT(SliderWidget2D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SliderWidget2D SliderWidget2D.cxx)
TARGET_LINK_LIBRARIES(SliderWidget2D vtkHybrid vtkWidgets)
