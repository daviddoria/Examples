cmake_minimum_required(VERSION 2.6)

PROJECT(PointsInsideBoxWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointsInsideBoxWidget PointsInsideBoxWidget.cxx)
TARGET_LINK_LIBRARIES(PointsInsideBoxWidget vtkHybrid vtkWidgets)
