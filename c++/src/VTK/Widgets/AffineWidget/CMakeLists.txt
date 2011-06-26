cmake_minimum_required(VERSION 2.6)

PROJECT(AffineWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AffineWidget AffineWidget.cxx)
TARGET_LINK_LIBRARIES(AffineWidget vtkHybrid vtkWidgets)
