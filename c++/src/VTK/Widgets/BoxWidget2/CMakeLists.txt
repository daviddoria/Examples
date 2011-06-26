cmake_minimum_required(VERSION 2.6)

PROJECT(BoxWidget2)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BoxWidget2 BoxWidget2.cxx)
TARGET_LINK_LIBRARIES(BoxWidget2 vtkHybrid vtkWidgets)
