cmake_minimum_required(VERSION 2.6)

PROJECT(SeedWidgetImage)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SeedWidgetImage SeedWidgetImage.cxx)
TARGET_LINK_LIBRARIES(SeedWidgetImage vtkHybrid vtkWidgets)
