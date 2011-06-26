cmake_minimum_required(VERSION 2.6)
PROJECT(DeleteCell)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DeleteCell DeleteCell.cxx)
TARGET_LINK_LIBRARIES(DeleteCell vtkHybrid)
