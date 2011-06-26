cmake_minimum_required(VERSION 2.6)

PROJECT(CircularLayout)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CircularLayout CircularLayout.cxx)
TARGET_LINK_LIBRARIES(CircularLayout vtkHybrid vtkInfovis vtkViews)

