cmake_minimum_required(VERSION 2.6)
PROJECT(ImageIterator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageIterator ImageIterator.cxx)
TARGET_LINK_LIBRARIES(ImageIterator vtkHybrid)

