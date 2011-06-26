cmake_minimum_required(VERSION 2.6)

PROJECT(CompassWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CompassWidget CompassWidget.cxx)
TARGET_LINK_LIBRARIES(CompassWidget vtkHybrid vtkInfovis vtkWidgets)
