cmake_minimum_required(VERSION 2.6)

PROJECT(ImplicitPlaneWidget2)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImplicitPlaneWidget2 ImplicitPlaneWidget2.cxx)
TARGET_LINK_LIBRARIES(ImplicitPlaneWidget2 vtkHybrid vtkWidgets)
