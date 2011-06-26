cmake_minimum_required(VERSION 2.6)

PROJECT(ColorVerticesByArray)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ColorVerticesByArray ColorVerticesByArray.cxx)
TARGET_LINK_LIBRARIES(ColorVerticesByArray vtkHybrid vtkInfovis vtkViews)
