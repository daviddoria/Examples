cmake_minimum_required(VERSION 2.6)

PROJECT(ImagePlaneWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImagePlaneWidget ImagePlaneWidget.cxx)
TARGET_LINK_LIBRARIES(ImagePlaneWidget vtkHybrid vtkWidgets)
