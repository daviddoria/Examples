cmake_minimum_required(VERSION 2.6)
PROJECT(CellLinks)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CellLinks CellLinks.cxx)
TARGET_LINK_LIBRARIES(CellLinks vtkHybrid)

