cmake_minimum_required(VERSION 2.6)

PROJECT(ScatterPlot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ScatterPlot ScatterPlot.cxx)

TARGET_LINK_LIBRARIES(ScatterPlot vtkHybrid vtkCharts)
