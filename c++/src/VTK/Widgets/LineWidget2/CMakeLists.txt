cmake_minimum_required(VERSION 2.6)

PROJECT(LineWidget2)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LineWidget2 LineWidget2.cxx)
TARGET_LINK_LIBRARIES(LineWidget2 vtkHybrid vtkWidgets)
