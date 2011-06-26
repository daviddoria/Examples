cmake_minimum_required(VERSION 2.6)

PROJECT(PlaneWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PlaneWidget PlaneWidget.cxx)
TARGET_LINK_LIBRARIES(PlaneWidget vtkHybrid vtkWidgets)
