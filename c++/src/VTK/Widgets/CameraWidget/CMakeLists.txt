cmake_minimum_required(VERSION 2.6)

PROJECT(CameraWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CameraWidget CameraWidget.cxx)
TARGET_LINK_LIBRARIES(CameraWidget vtkHybrid vtkWidgets)
