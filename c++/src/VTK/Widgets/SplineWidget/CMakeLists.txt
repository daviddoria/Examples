cmake_minimum_required(VERSION 2.6)

PROJECT(SplineWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SplineWidget SplineWidget.cxx)
TARGET_LINK_LIBRARIES(SplineWidget vtkHybrid vtkWidgets)
