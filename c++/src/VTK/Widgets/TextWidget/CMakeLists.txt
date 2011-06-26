cmake_minimum_required(VERSION 2.6)

PROJECT(TextWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TextWidget TextWidget.cxx)
TARGET_LINK_LIBRARIES(TextWidget vtkHybrid vtkWidgets)
