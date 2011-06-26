cmake_minimum_required(VERSION 2.6)

PROJECT(StackedPlot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StackedPlot StackedPlot.cxx)

TARGET_LINK_LIBRARIES(StackedPlot vtkHybrid vtkCharts)
