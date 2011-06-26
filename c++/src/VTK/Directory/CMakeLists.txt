cmake_minimum_required(VERSION 2.6)
PROJECT(Directory)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Directory Directory.cxx)
TARGET_LINK_LIBRARIES(Directory vtkHybrid)

