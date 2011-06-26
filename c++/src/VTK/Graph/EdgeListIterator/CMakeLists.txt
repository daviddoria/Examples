cmake_minimum_required(VERSION 2.6)

PROJECT(EdgeListIterator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(EdgeListIterator EdgeListIterator.cxx)
TARGET_LINK_LIBRARIES(EdgeListIterator vtkHybrid vtkInfovis)

