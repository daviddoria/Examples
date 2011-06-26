cmake_minimum_required(VERSION 2.6)

PROJECT(TreeIteratorComparison)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TreeIteratorComparison TreeIteratorComparison.cxx)
TARGET_LINK_LIBRARIES(TreeIteratorComparison vtkHybrid vtkInfovis)
