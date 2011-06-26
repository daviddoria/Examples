cmake_minimum_required(VERSION 2.6)

PROJECT(HandleWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HandleWidget HandleWidget.cxx)
TARGET_LINK_LIBRARIES(HandleWidget vtkHybrid vtkWidgets)
