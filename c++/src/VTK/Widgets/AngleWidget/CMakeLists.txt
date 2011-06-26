cmake_minimum_required(VERSION 2.6)

PROJECT(AngleWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Default Default.cxx)
TARGET_LINK_LIBRARIES(Default vtkHybrid vtkWidgets)

ADD_EXECUTABLE(AngleWidget AngleWidget.cxx)
TARGET_LINK_LIBRARIES(AngleWidget vtkHybrid vtkWidgets)
