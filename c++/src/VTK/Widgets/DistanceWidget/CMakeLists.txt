cmake_minimum_required(VERSION 2.6)

PROJECT(DistanceWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DistanceWidget DistanceWidget.cxx)
TARGET_LINK_LIBRARIES(DistanceWidget vtkHybrid vtkWidgets)
