cmake_minimum_required(VERSION 2.6)

PROJECT(ParallelCoordinatesView)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ParallelCoordinatesView ParallelCoordinatesView.cxx)
TARGET_LINK_LIBRARIES(ParallelCoordinatesView vtkHybrid vtkViews)
