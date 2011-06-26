cmake_minimum_required(VERSION 2.6)
PROJECT(RectilinearGrid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RectilinearGrid RectilinearGrid.cxx)
TARGET_LINK_LIBRARIES(RectilinearGrid vtkHybrid)

#ADD_EXECUTABLE(VisualizeCells VisualizeCells.cxx)
#TARGET_LINK_LIBRARIES(VisualizeCells vtkHybrid)

