cmake_minimum_required(VERSION 2.6)

PROJECT(DepthFirstSearchAnimation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DepthFirstSearchAnimation DepthFirstSearchAnimation.cxx)
TARGET_LINK_LIBRARIES(DepthFirstSearchAnimation vtkHybrid vtkInfovis vtkViews)
