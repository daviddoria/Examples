cmake_minimum_required(VERSION 2.6)

PROJECT(AxesIndicator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AxesIndicator AxesIndicator.cpp)
TARGET_LINK_LIBRARIES(AxesIndicator vtkHybrid vtkWidgets)
