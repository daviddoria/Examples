cmake_minimum_required(VERSION 2.6)

PROJECT(Tree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Tree Tree.cxx)
TARGET_LINK_LIBRARIES(Tree vtkHybrid vtkInfovis vtkViews)

