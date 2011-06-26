cmake_minimum_required(VERSION 2.6)

PROJECT(HoverWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HoverWidget HoverWidget.cxx)
TARGET_LINK_LIBRARIES(HoverWidget vtkHybrid vtkWidgets)
