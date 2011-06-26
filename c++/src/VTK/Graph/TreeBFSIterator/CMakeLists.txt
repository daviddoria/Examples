cmake_minimum_required(VERSION 2.6)

PROJECT(TreeBFSIterator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TreeBFSIterator TreeBFSIterator.cxx)
TARGET_LINK_LIBRARIES(TreeBFSIterator vtkHybrid vtkInfovis)
