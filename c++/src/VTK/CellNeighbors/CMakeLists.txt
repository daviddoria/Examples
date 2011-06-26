cmake_minimum_required(VERSION 2.6)

PROJECT(CellNeighbors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointNeighbors PointNeighbors.cxx)
TARGET_LINK_LIBRARIES(PointNeighbors vtkHybrid)

ADD_EXECUTABLE(EdgeNeighbors EdgeNeighbors.cxx)
TARGET_LINK_LIBRARIES(EdgeNeighbors vtkHybrid)
