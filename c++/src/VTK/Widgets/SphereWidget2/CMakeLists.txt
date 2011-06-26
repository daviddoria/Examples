cmake_minimum_required(VERSION 2.6)

PROJECT(SphereWidget2)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SphereWidget2 SphereWidget2.cxx)
TARGET_LINK_LIBRARIES(SphereWidget2 vtkHybrid vtkWidgets)
