cmake_minimum_required(VERSION 2.6)

PROJECT(ContinuousValueWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ContinuousValueWidget ContinuousValueWidget.cxx)
TARGET_LINK_LIBRARIES(ContinuousValueWidget vtkHybrid vtkWidgets)
