cmake_minimum_required(VERSION 2.6)

PROJECT(BarGraph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BarGraph BarGraph.cxx)

TARGET_LINK_LIBRARIES(BarGraph vtkHybrid vtkCharts)
