cmake_minimum_required(VERSION 2.6)

PROJECT(ParallelCoordinates)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ParallelCoordinates ParallelCoordinates.cxx)
TARGET_LINK_LIBRARIES(ParallelCoordinates vtkHybrid vtkCharts)

ADD_EXECUTABLE(parse parse.cxx)
TARGET_LINK_LIBRARIES(parse)

ADD_EXECUTABLE(orig orig.cxx)
TARGET_LINK_LIBRARIES(orig vtkHybrid vtkCharts)

ADD_EXECUTABLE(simple simple.cxx)
TARGET_LINK_LIBRARIES(simple vtkHybrid vtkCharts)
