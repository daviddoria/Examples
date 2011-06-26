cmake_minimum_required(VERSION 2.6)

PROJECT(SphereWidget)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SphereWidget SphereWidget.cxx)
TARGET_LINK_LIBRARIES(SphereWidget vtkHybrid vtkWidgets)
