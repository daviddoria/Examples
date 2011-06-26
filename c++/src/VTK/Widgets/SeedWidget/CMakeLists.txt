cmake_minimum_required(VERSION 2.6)

PROJECT(SeedWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SeedWidget SeedWidget.cxx)
TARGET_LINK_LIBRARIES(SeedWidget vtkHybrid vtkWidgets)
