cmake_minimum_required(VERSION 2.6)

PROJECT(ContourWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ContourWidget ContourWidget.cxx)
TARGET_LINK_LIBRARIES(ContourWidget vtkHybrid vtkWidgets)
