cmake_minimum_required(VERSION 2.6)

PROJECT(LogoWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LogoWidget LogoWidget.cxx)
TARGET_LINK_LIBRARIES(LogoWidget vtkHybrid vtkWidgets)
