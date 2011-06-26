cmake_minimum_required(VERSION 2.6)

PROJECT(TwoWidgets)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TwoWidgets TwoWidgets.cxx)
TARGET_LINK_LIBRARIES(TwoWidgets vtkHybrid vtkWidgets)
