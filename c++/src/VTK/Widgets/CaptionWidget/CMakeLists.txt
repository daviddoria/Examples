cmake_minimum_required(VERSION 2.6)

PROJECT(CaptionWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CaptionWidget CaptionWidget.cxx)
TARGET_LINK_LIBRARIES(CaptionWidget vtkHybrid vtkInfovis vtkWidgets)
