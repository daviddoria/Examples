cmake_minimum_required(VERSION 2.6)

PROJECT(BoostBreadthFirstSearchTree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BoostBreadthFirstSearchTree BoostBreadthFirstSearchTree.cxx)
TARGET_LINK_LIBRARIES(BoostBreadthFirstSearchTree vtkHybrid vtkInfovis vtkViews)
