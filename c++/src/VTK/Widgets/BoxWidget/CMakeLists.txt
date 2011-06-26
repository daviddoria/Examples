cmake_minimum_required(VERSION 2.6)

PROJECT(BoxWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BoxWidget BoxWidget.cxx)
TARGET_LINK_LIBRARIES(BoxWidget vtkHybrid vtkWidgets)
